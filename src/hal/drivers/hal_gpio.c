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

// Probably enough, just to avoid krealloc every time we change "chips"
#define MAX_CHIPS 8
// There isn't really any limit
#define MAX_CHAN 128

char *inputs[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(inputs, MAX_CHAN, "list of pins to use for input");
char *outputs[MAX_CHAN];
RTAPI_MP_ARRAY_STRING(outputs, MAX_CHAN, "list of pins to use for output");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* this structure contains the runtime data needed by the
   driver for a single port/channel
*/

typedef struct{
	hal_u32_t *mode;
	hal_bit_t *value;
} hal_gpio_hal_t;
    
typedef struct {
	int num_lines;
    hal_gpio_hal_t **hal;
    struct gpiod_chip *chip;
    struct gpiod_line_bulk *bulk;
} hal_gpio_bulk_t;

typedef struct {
	// Bulk line access has to all be to the same "chip" so we have an
	// array of chps with their bulk line collections. 
	int num_in_chips;
	int num_out_chips;
    hal_gpio_bulk_t in_chips[MAX_CHIPS];
    hal_gpio_bulk_t out_chip[MAX_CHIPS];
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

int rtapi_app_main(void){
    int retval;
    int i, c;
    char hal_name[HAL_NAME_LEN];
    struct gpiod_line *temp_line;
    struct gpiod_chip *temp_chip;

    comp_id = hal_init("hal_gpio");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "hal_gpio: ERROR: hal_init() failed\n");
        goto fail0;
    }

    // allocate shared memory for the base struct
    gpio = (hal_gpio_t *)rtapi_kmalloc(sizeof(hal_gpio_t), RTAPI_GFP_KERNEL);
    if (gpio == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,
                "hal_gpio component: Out of Memory\n");
        goto fail0;
    }

	gpio->num_in_chips = -1;
	gpio->num_out_chips = -1;
    for (i = 0; inputs[i]; i++) {
		temp_line = gpiod_line_find(inputs[i]);
		if (temp_line <= 0) {
			rtapi_print_msg(RTAPI_MSG_ERR, "The GPIO line %s can not be found\n", inputs[i]);
			goto fail0;
		}
		temp_chip = gpiod_line_get_chip(temp_line);
		rtapi_print("Chip %s\n", gpiod_chip_name(temp_chip));
		for (c = 0; c < gpio->num_in_chips
				&& strcmp(gpiod_chip_name(gpio->in_chips[c].chip), gpiod_chip_name(temp_chip)) == 0; c++){}

		rtapi_print("so far c = %i\n", c);
		if (c > gpio->num_in_chips){
			gpio->in_chips[c].chip = temp_chip;
			gpio->in_chips[c].num_lines = 0;
			gpiod_line_bulk_init(gpio->in_chips[c].bulk);
		}
	}
    /*
    for (int i = 0; i < gpio->num_inputs; i++){
	gpio->inputs[i].line = gpiod_line_find(inputs[i]);
	if (gpio->inputs[i].line <= 0) {

	}
	gpiod_line_request_input(gpio->inputs[i].line, "linuxcnc");
	gpio->inputs[i].hal = hal_malloc(sizeof(hal_gpio_hal_t));
	retval = hal_pin_bit_newf(HAL_OUT, &(gpio->inputs[i].hal->value), comp_id, "hal_gpio.%s.in", inputs[i]);
    }
    for (int i = 0; i < gpio->num_outputs; i++){
	gpio->outputs[i].line = gpiod_line_find(outputs[i]);
	if (gpio->outputs[i].line <= 0) {
	    rtapi_print_msg(RTAPI_MSG_ERR, "The GPIO line %s can not be found\n", outputs[i]);
	    goto fail0;
	}
	gpiod_line_request_output(gpio->outputs[i].line, "linuxcnc", 0);
	gpio->outputs[i].hal = hal_malloc(sizeof(hal_gpio_hal_t));
	retval = hal_pin_bit_newf(HAL_IN, &(gpio->outputs[i].hal->value), comp_id, "hal_gpio.%s.out", outputs[i]);
    }
    */
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
    hal_exit(comp_id);
    return -1;

}
/**************************************************************
* REALTIME PORT READ/WRITE FUNCTION                                *
**************************************************************/

static void hal_gpio_read(void *arg, long period)
{
    //hal_gpio_t *gpio = arg;
    //int i;

}

static void hal_gpio_write(void *arg, long period)
{
   // hal_gpio_t *gpio = arg;
   // int i;

}
