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
 * @file hg_input_rc.c
 * Minimal application reading rc input values from uORB service
 *
 * @author Leutrim Mustafa
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>            // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/input_rc.h> // uORB for input_rc

__EXPORT int hg_rcinput_main(int argc, char *argv[]);

int hg_rcinput_main(int argc, char *argv[])
{
    PX4_INFO("Hello Hovergames RCINPUT!");

    int input_rc_sub = orb_subscribe(ORB_ID(input_rc)); // subscribe ORB ID
    orb_set_interval(input_rc_sub, 200);                // set the intervall

    px4_pollfd_struct_t fds_input_rc;
    fds_input_rc.fd = input_rc_sub;
    fds_input_rc.events = POLLIN;

    int counter = 20;

    for (int i = 1; i <= counter; i++)
    {
        int poll_ret = px4_poll(&fds_input_rc, 1, 1000);

        if (poll_ret == 0)
        {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");
        }
        else
        {

            if (fds_input_rc.revents & POLLIN)
            {
                /* obtained data for the first file descriptor */
                struct input_rc_s input_rc;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(input_rc), input_rc_sub, &input_rc);

                /* print at the first timethe channel count and a head for the table */
                if (i == 1)
                {
                    printf("channel_count: %i\n", input_rc.channel_count);

                    for (uint j = 0; j < input_rc.channel_count; j++)
                    {
                        printf("CH%02i  ", j);
                    }
                    printf("\n");
                }

                /* print for ever channel the value */
                for (uint j = 0; j < input_rc.channel_count; j++)
                {
                    printf("%04i  ", input_rc.values[j]);
                }
                printf("\n");
            }
        }
    }

    PX4_INFO("Hovergames RCINPUT exit"); // print in console 

    return 0;
}
