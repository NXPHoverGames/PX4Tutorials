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
 * @file hg_battery.c
 * Minimal application reading battery values from uORB service
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
#include <uORB/topics/battery_status.h> // uORB for battery status

__EXPORT int hg_battery_main(int argc, char *argv[]);

int hg_battery_main(int argc, char *argv[])
{
    // print in console 
	PX4_INFO("Hello Hovergames BATTERY!");

    //subscribe ORB ID
	int battery_sub = orb_subscribe(ORB_ID(battery_status));

    // set the intervall 
    orb_set_interval(battery_sub, 200);                      

    px4_pollfd_struct_t fds_battery;
    fds_battery.fd = battery_sub;
    fds_battery.events = POLLIN;

    for (int i = 0; i < 5; i++)
    {
        int poll_ret = px4_poll(&fds_battery, 1, 1000);

        if (poll_ret == 0)
        {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");
        }
        else
        {

            if (fds_battery.revents & POLLIN)
            {
                /* obtained data for the first file descriptor */
                struct battery_status_s battery;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(battery_status), battery_sub, &battery);

                PX4_INFO("VOLTAGE:\t %2.2f", (double)battery.voltage_v);
                PX4_INFO("CURRENT:\t %2.2f", (double)battery.current_a);
            }
        }
    }

    // print in console 
	PX4_INFO("Hovergames BATTERY exit"); 

	return 0;
}
