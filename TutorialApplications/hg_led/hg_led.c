/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *   Copyright 2019 NXP.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hg_led.c
 * Minimal application for led control
 *
 * @author Leutrim Mustafa
 */

#include <px4_config.h>	
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>                  // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/led_control.h>    // uORB for led_control

__EXPORT int hg_led_main(int argc, char *argv[]);	// export main for starting in other thread

int hg_led_main(int argc, char *argv[])
{
	// print in console 
    PX4_INFO("Hello Hovergames LED");
    
    // structur with led_control paramters
    struct led_control_s led_control;
    
    // fill the structure with 0
    memset(&led_control, 0, sizeof(led_control));
    
    // advertise structure for ORB_ID
    orb_advert_t led_control_pub = orb_advertise(ORB_ID(led_control), &led_control); 

    led_control.num_blinks = 10;                     	// blinks
    led_control.priority = LED_CONTROL_MAX_PRIORITY; 	// priority
    led_control.mode = LED_CONTROL_MODE_BLINK_NORMAL;  	// led mode
    led_control.led_mask = 0xff;                     	// select leds - 0xff for all
    led_control.color = LED_CONTROL_COLOR_GREEN;     	// color

    // publish the message to uORB service
    orb_publish(ORB_ID(led_control), led_control_pub, &led_control); 

    // print in console 
    PX4_INFO("Hovergames LED exit"); 

    return 0;	// return of main function	
}
