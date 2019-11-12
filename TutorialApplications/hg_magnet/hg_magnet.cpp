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
 * @file hg_accel.c
 * Minimal application reading megnetometer values from uORB topic
 *
 * @author Leutrim Mustafa
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/uORB.h>                          // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/vehicle_local_position.h> // uORB topic for accelerometer sensor data
#include <uORB/topics/led_control.h>            // uORB topic for LED control

using matrix::wrap_2pi;

extern "C" __EXPORT int hg_magnet_main(int argc, char *argv[]);

int hg_magnet_main(int argc, char *argv[])
{
    PX4_INFO("Hello HoverGames MAG!"); // print in console

    struct led_control_s led_control;             // structure with led_control parameters
    memset(&led_control, 0, sizeof(led_control)); // fill the structure with 0

    orb_advert_t led_control_pub = orb_advertise(ORB_ID(led_control), &led_control); // advertise led_control topic

    led_control.num_blinks = led_control_s::MAX_PRIORITY; // blinks
    led_control.priority = 2;                             // priority
    led_control.mode = led_control_s::MODE_BLINK_FAST;    // LED mode
    led_control.led_mask = 0xff;                          // select LEDs - 0xff for all
    led_control.color = led_control_s::COLOR_RED;         // color

    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position)); // subscribe to uORB topic
    orb_set_interval(vehicle_local_position_sub, 200);                              // set the interval to 200 ms

    px4_pollfd_struct_t fd_vehicle_local_position;
    fd_vehicle_local_position.fd = vehicle_local_position_sub;
    fd_vehicle_local_position.events = POLLIN;

    int counter = 255;

    printf("%02i | heading \n", counter);
    printf("---------------\n");

    for (int i = 1; i <= counter; i++)
    {
        int poll_ret = px4_poll(&fd_vehicle_local_position, 1, 1000); // wait for sensor update of 1 file descriptor for 1000 ms (1 second)

        if (poll_ret == 0) // this means none of our providers is giving us data
        {
            PX4_ERR("Got no data within a second");
        }
        else
        {
            if (fd_vehicle_local_position.revents & POLLIN)
            {
                struct vehicle_local_position_s vehicle_local_position_st;                                        // obtained data for the first file descriptor
                orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &vehicle_local_position_st); // copy data into local buffer

                int16_t heading = (int16_t)math::degrees(wrap_2pi(vehicle_local_position_st.yaw)); // calculate orientation in dagrees

                printf("%02i |   %0.3i    | ", i, heading);

                if (heading < 10 || heading > 350)
                {
                    printf("NORTH |\n");
                    orb_publish(ORB_ID(led_control), led_control_pub, &led_control); // publish the message to uORB topic
                }
                else
                {
                    printf("      |\n");
                }
            }
        }
    }

    PX4_INFO("HoverGames MAG exit"); // print in console 

    return 0;
}