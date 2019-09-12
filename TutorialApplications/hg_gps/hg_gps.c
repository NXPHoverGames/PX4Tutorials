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
 * @file hg_gps.c
 * Minimal application reading gps position from uORB service
 *
 * @author Leutrim Mustafa
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>                // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/vehicle_gps_position.h> // uORB for vehicle_gps_position
#include <uORB/topics/ekf_gps_position.h> // uORB for vehicle_gps_position

__EXPORT int hg_gps_main(int argc, char *argv[]);

int hg_gps_main(int argc, char *argv[])
{
    PX4_INFO("Hello Hovergames GPS!");

    int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position)); //subscribe ORB ID
    orb_set_interval(vehicle_gps_position_sub, 200);                    // set the intervall

    px4_pollfd_struct_t fds_vehicle_gps_position;
    fds_vehicle_gps_position.fd = vehicle_gps_position_sub;
    fds_vehicle_gps_position.events = POLLIN;

    int counter = 20;

    printf("vehicle_gps_position\n");
    printf("%02i | latitude | longitude | altitude\n", counter);
    printf("---------------------------------------\n");

    for (int i = 1; i <= counter; i++)
    {
        int poll_ret = px4_poll(&fds_vehicle_gps_position, 1, 1000);

        if (poll_ret == 0)
        {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");
        }
        else
        {

            if (fds_vehicle_gps_position.revents & POLLIN)
            {
                /* obtained data for the first file descriptor */
                struct vehicle_gps_position_s vehicle_gps_position;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &vehicle_gps_position);

                printf("%02i |  %i  |  %i  |  %i  \n", i, vehicle_gps_position.lat, vehicle_gps_position.lon, vehicle_gps_position.alt);
            }
        }
    }

    int ekf_gps_position_sub = orb_subscribe(ORB_ID(ekf_gps_position)); //subscribe ORB ID
    orb_set_interval(ekf_gps_position_sub, 200);                    // set the intervall

    px4_pollfd_struct_t fds_ekf_gps_position;
    fds_ekf_gps_position.fd = ekf_gps_position_sub;
    fds_ekf_gps_position.events = POLLIN;


    printf("ekf_gps_position\n");
    printf("%02i | latitude | longitude | altitude\n", counter);
    printf("---------------------------------------\n");

    for (int i = 1; i <= counter; i++)
    {
        int poll_ret = px4_poll(&fds_ekf_gps_position, 1, 1000);

        if (poll_ret == 0)
        {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");
        }
        else
        {

            if (fds_ekf_gps_position.revents & POLLIN)
            {
                /* obtained data for the first file descriptor */
                struct ekf_gps_position_s ekf_gps_position;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(ekf_gps_position), ekf_gps_position_sub, &ekf_gps_position);

                printf("%02i |  %i  |  %i  |  %i  \n", i, ekf_gps_position.lat, ekf_gps_position.lon, ekf_gps_position.alt);
            }
        }
    }

    PX4_INFO("Hovergames GPS exit"); // print in console 

    return 0;
}
