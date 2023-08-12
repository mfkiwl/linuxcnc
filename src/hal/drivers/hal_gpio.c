/********************************************************************
* Description:  hal_gpio.c
*               GPIO driver for Rapberry Pi and similar using the
* 		gpiod library
*
* Author: Andy Pugh
* License: GPL Version 2+
*
* Copyright (c) 2023 All rights reserved.
*
*********************************************************************
    This program is free software; you can redistribute it and/or
    modify it under the terms of version 2 of the GNU General
    Public License as published by the Free Software Foundation.
    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    THE AUTHORS OF THIS LIBRARY ACCEPT ABSOLUTELY NO LIABILITY FOR
    ANY HARM OR LOSS RESULTING FROM ITS USE.  IT IS _EXTREMELY_ UNWISE
    TO RELY ON SOFTWARE ALONE FOR SAFETY.  Any machinery capable of
    harming persons must have provisions for completely removing power
    from all motors, etc, before persons enter any danger area.  All
    machinery must be designed to comply with local and national safety
    codes, and the authors of this software can not, and do not, take
    any responsibility for such compliance.

    This code was written as part of the LinuxCNC project.  For more
    information, go to www.linuxcnc.org.
*/

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_slab.h"  	/* kmalloc() */
#include "hal.h"		/* HAL public API decls */
#include <gpiod.h>
#include <string.h>


MODULE_AUTHOR("Andy Pugh");
MODULE_DESCRIPTION("GPIO driver using gpiod / libgpiod");
MODULE_LICENSE("GPL");


// There isn't really any limit except for in the MP_ARRAY macros. 
#define MAX_CHAN 128

char *inputs[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(inputs, MAX_CHAN, "list of pins to use for input")
char *outputs[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(outputs, MAX_CHAN, "list of pins to use for output")
char *invert[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(invert, MAX_CHAN, "set as inverted")
char *reset[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(reset, MAX_CHAN, "add to reset list")
char *opendrain[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(opendrain, MAX_CHAN, "set OPEN_DRAIN flag")
char *opensource[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(opensource, MAX_CHAN, "set OPEN_SOURCE flag")
char *biasdisable[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(biasdisable, MAX_CHAN, "set BIAS_DISABLE flag")
char *pulldown[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(pulldown, MAX_CHAN, "set BIAS_PULL_DOWN flag")
char *pullup[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(pullup, MAX_CHAN, "set BIAS_PULL_UP flag")

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct{
    hal_bit_t *value;
} hal_gpio_hal_t;

/* flags are defined such:
 * bits 1 - 4 are gpiod flags
 * OPEN_DRAIN		= BIT(0) - not currently supported
 * OPEN_SOURCE		= BIT(1) - not currently supported
 * BIAS_DISABLE 	= BIT(2) - not currently supported
 * PULL_DOWN		= BIT(3) - not currently supported
 * PULL_UP		= BIT(4) - not currently supported
 *
 * hal_gpio flags
 * INVERT 		= BIT(5)
 * RESET		= BIT(6) - not currently supported
 */

typedef struct {
    int num_lines;
    int *vals;
    int *flags;
    hal_gpio_hal_t *hal;
    struct gpiod_chip *chip;
    struct gpiod_line_bulk *bulk;
} hal_gpio_bulk_t;

typedef struct {
    // Bulk line access has to all be to the same "chip" so we have an
    // array of chips with their bulk line collections. 
    int num_in_chips;
    int num_out_chips;
    hal_gpio_bulk_t *in_chips;
    hal_gpio_bulk_t *out_chips;
} hal_gpio_t;

static int comp_id;
static hal_gpio_t *gpio;

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

static void hal_gpio_read(void *arg, long period);
static void hal_gpio_write(void *arg, long period);

/***********************************************************************
*                      SETUP AND EXIT CODE                             *
************************************************************************/

int flags(char *name){
    int f = 0;
    int i;
    for (i = 0; opendrain[i]; i++)
	if (strcmp(name, opendrain[i]) == 0) f |= 0x1;
    for (i = 0; opensource[i]; i++)
	if (strcmp(name, opensource[i]) == 0) f |= 0x2;
    for (i = 0; biasdisable[i]; i++)
	if (strcmp(name, biasdisable[i]) == 0) f |= 0x4;
    for (i = 0; pulldown[i]; i++)
	if (strcmp(name, pulldown[i]) == 0) f |= 0x8;
    for (i = 0; pullup[i]; i++)
	if (strcmp(name, pullup[i]) == 0) f |= 0x10;
    for (i = 0; invert[i]; i++)
	if (strcmp(name, invert[i]) == 0) f |= 0x20;
    for (i = 0; reset[i]; i++)
	if (strcmp(name, reset[i]) == 0) f |= 0x40;
    rtapi_print_msg(RTAPI_MSG_INFO,"line %s flags %02x\n", name, f);
    return f;
}


int build_chips_collection(char *name, hal_gpio_bulk_t **ptr, int *count){
    int c;
    static struct gpiod_chip *temp_chip;
    struct gpiod_line *temp_line;
    
    temp_line = gpiod_line_find(name);
    if (temp_line <= 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "The GPIO line %s can not be found\n", name);
	    return -EINVAL;
    }
    temp_chip = gpiod_line_get_chip(temp_line);
    for (c = 0; c < *count 
		&& (strcmp(gpiod_chip_name((*ptr)[c].chip), gpiod_chip_name(temp_chip))
		// max of 64 lines per bulk, so carry on to another "chip" if full
		||  (*ptr)[c].num_lines > 3);
		c++){
    }

    if (c >= *count){
	    (*count)++;
rtapi_print("*ptr is at %p\n", (*ptr));
	    *ptr = rtapi_krealloc(*ptr, sizeof(hal_gpio_bulk_t) * (*count), RTAPI_GFP_KERNEL);
rtapi_print("*ptr is at %p\n", (*ptr));
	    (*ptr)[c].chip = gpiod_line_get_chip(temp_line);
	    rtapi_print_msg(RTAPI_MSG_INFO, "hal_gpio: added chip %s index %i\n", gpiod_chip_name((*ptr)[c].chip), c);
	    (*ptr)[c].num_lines = 0;
	    (*ptr)[c].bulk = rtapi_kmalloc(sizeof(*(*ptr)[c].bulk), RTAPI_GFP_KERNEL);
	    gpiod_line_bulk_init((*ptr)[c].bulk);
    }
    rtapi_print_msg(RTAPI_MSG_INFO, "hal_gpio: adding IO line %s to chip %i\n", name, c);
    temp_line = gpiod_chip_find_line((*ptr)[c].chip, name);
    (*ptr)[c].num_lines++;
rtapi_print("flags is at %p\n", (*ptr)[c].flags );
    (*ptr)[c].flags = rtapi_krealloc((*ptr)[c].flags, (*ptr)[c].num_lines * sizeof(int), RTAPI_GFP_KERNEL);
rtapi_print("flags is at %p\n", (*ptr)[c].flags );
    //(*ptr)[c].flags[(*ptr)[c].num_lines - 1] = flags(name);
    //gpiod_line_set_flags(temp_line, (*ptr)[c].flags);
rtapi_print("vals is at %p\n", (*ptr)[c].vals );
    (*ptr)[c].vals = rtapi_krealloc((*ptr)[c].vals, (*ptr)[c].num_lines * sizeof(int), RTAPI_GFP_KERNEL);
rtapi_print("vals is at %p\n", (*ptr)[c].vals );
    gpiod_line_bulk_add((*ptr)[c].bulk, temp_line);
    //gpiod_chip_close(temp_chip);

    return 0;
}
    
int rtapi_app_main(void){
    int retval = 0;
    int i, c;
    char hal_name[HAL_NAME_LEN];
    const char *line_name;

    rtapi_set_msg_level(5);
    
    comp_id = hal_init("hal_gpio");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: ERROR: hal_init() failed\n");
        goto fail0;
    }

    // allocate shared memory for the base struct
    gpio = rtapi_kmalloc(sizeof(hal_gpio_t), RTAPI_GFP_KERNEL);
    if (gpio == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "hal_gpio component: Out of Memory\n");
        goto fail0;
    }

    gpio->num_in_chips = 0;
    gpio->num_out_chips = 0;
    for (i = 0; inputs[i]; i++) {
	retval = build_chips_collection(inputs[i], &gpio->in_chips, &gpio->num_in_chips);
	if (retval < 0) goto fail0;
    }
    for (c = 0; c < gpio->num_in_chips; c++){
	if (gpiod_line_request_bulk_input(gpio->in_chips[c].bulk, "linuxcnc") < 0){
	    rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: Failed to register input pin collection\n");
	    goto fail0;
	}
	gpio->in_chips[c].hal = hal_malloc(gpio->in_chips[c].num_lines * sizeof(hal_gpio_hal_t));
	for (i = 0; i < gpio->in_chips[c].num_lines; i++){
	    line_name = gpiod_line_name(gpiod_line_bulk_get_line(gpio->in_chips[c].bulk, i));
	    retval += hal_pin_bit_newf(HAL_OUT, &(gpio->in_chips[c].hal[i].value), comp_id, "hal_gpio.%s-in", line_name);
	}
	if (retval < 0){
	    rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: Failed to allocate GPIO input HAL pins\n");
	    goto fail0;
	}
	    
    }
    
    for (i = 0; outputs[i]; i++) {
	retval = build_chips_collection(outputs[i], &gpio->out_chips, &gpio->num_out_chips);
	if (retval < 0) goto fail0;
    }
    for (c = 0; c < gpio->num_out_chips; c++){
	if (gpiod_line_request_bulk_output(gpio->out_chips[c].bulk, "linuxcnc", gpio->out_chips[c].vals) < 0){
	    rtapi_print_msg(RTAPI_MSG_ERR, "Failed to register output pin collection\n");
	    goto fail0;
       }
	gpio->out_chips[c].hal = hal_malloc(gpio->out_chips[c].num_lines * sizeof(hal_gpio_hal_t));
	for (i = 0; i < gpio->out_chips[c].num_lines; i++){
	    line_name = gpiod_line_name(gpiod_line_bulk_get_line(gpio->out_chips[c].bulk, i));
	    retval += hal_pin_bit_newf(HAL_IN, &(gpio->out_chips[c].hal[i].value), comp_id, "hal_gpio.%s-out", line_name);
	}
	if (retval < 0){
	    rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: Failed to allocate GPIO output HAL pins\n");
	    goto fail0;
	}
    }

    rtapi_snprintf(hal_name, HAL_NAME_LEN, "hal_gpio.read");
    retval = hal_export_funct(hal_name, hal_gpio_read, gpio, 0, 0, comp_id);
    rtapi_snprintf(hal_name, HAL_NAME_LEN, "hal_gpio.write");
    retval = hal_export_funct(hal_name, hal_gpio_write, gpio, 0, 0, comp_id);
    if (retval < 0){
	rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: failed to export functions\n");
	goto fail0;
    }
    hal_ready(comp_id);
    return 0;

    fail0:
    for (c = 0; c < gpio->num_in_chips; c++){
	gpiod_chip_close(gpio->in_chips[c].chip);
	rtapi_kfree(gpio->in_chips[c].chip);
	rtapi_kfree(gpio->in_chips[c].bulk);}
    for (c = 0; c < gpio->num_out_chips; c++){
	gpiod_chip_close(gpio->out_chips[c].chip);
	rtapi_kfree(gpio->out_chips[c].chip);
	rtapi_kfree(gpio->out_chips[c].bulk);}
    rtapi_kfree(gpio->in_chips);
    rtapi_kfree(gpio->out_chips);
    rtapi_kfree(gpio);
    hal_exit(comp_id);
    return -1;

}

/**************************************************************
* REALTIME PORT READ/WRITE FUNCTION                                *
**************************************************************/

static void hal_gpio_read(void *arg, long period)
{
    hal_gpio_t *gpio = arg;
    int i, c;
    for (c = 0; c < gpio->num_in_chips; c++){
	gpiod_line_get_value_bulk(gpio->in_chips[c].bulk, gpio->in_chips[c].vals);
	for (i = 0; i < gpio->in_chips[c].num_lines; i++){
	   *(gpio->in_chips[c].hal[i].value) = gpio->in_chips[c].vals[i];
	}
    }
}

static void hal_gpio_write(void *arg, long period)
{
    hal_gpio_t *gpio = arg;
    int i, c;
    for (c = 0; c < gpio->num_out_chips; c++){
	for (i = 0; i < gpio->out_chips[c].num_lines; i++){
	   gpio->out_chips[c].vals[i] = *(gpio->out_chips[c].hal[i].value);
	}
	gpiod_line_set_value_bulk(gpio->out_chips[c].bulk, gpio->out_chips[c].vals);
    }
}

void rtapi_app_exit(void) {
    int c;
    for (c = 0; c < gpio->num_in_chips; c++){
	gpiod_chip_close(gpio->in_chips[c].chip);
	rtapi_kfree(gpio->in_chips[c].chip);
	rtapi_kfree(gpio->in_chips[c].bulk);}
    for (c = 0; c < gpio->num_out_chips; c++){
	gpiod_chip_close(gpio->out_chips[c].chip);
	rtapi_kfree(gpio->out_chips[c].chip);
	rtapi_kfree(gpio->out_chips[c].bulk);}
    rtapi_kfree(gpio->in_chips);
    rtapi_kfree(gpio->out_chips);
    rtapi_kfree(gpio);
    hal_exit(comp_id);
}

