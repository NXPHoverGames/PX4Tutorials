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
 * @file hg_gyro.c
 * Minimal application reading gyro values from uORB service
 *
 * @author Leutrim Mustafa
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>               // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/sensor_gyro.h> // uORB for gyro

__EXPORT int hg_gyro_main(int argc, char *argv[]);

int hg_gyro_main(int argc, char *argv[])
{
    PX4_INFO("Hello Hovergames gyro!");

    int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro)); //subscribe ORB ID
    orb_set_interval(gyro_sub, 200);                   // set the intervall

    px4_pollfd_struct_t fds_gyro;
    fds_gyro.fd = gyro_sub;
    fds_gyro.events = POLLIN;

    int counter = 20;
    printf("%02i |  gyro_x |  gyro_y |  gyro_z\n", counter);
    printf("-----------------------------------\n");

    for (int i = 1; i <= counter; i++)
    {
        int poll_ret = px4_poll(&fds_gyro, 1, 1000);

        if (poll_ret == 0)
        {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");
        }
        else
        {

            if (fds_gyro.revents & POLLIN)
            {
                /* obtained data for the first file descriptor */
                struct sensor_gyro_s gyro;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro);

                printf("%02i |  %+2.2f  |  %+2.2f  |  %+2.2f  \n", i, (double)gyro.x, (double)gyro.y, (double)gyro.z);
            }
        }
    }

    PX4_INFO("Hovergames gyro exit"); // print in console 

    return 0;
}
