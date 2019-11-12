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
 * @file hg_motortest.c
 * Minimal application for motortest
 *
 * @author Leutrim Mustafa
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>              // asynchronous messaging API used for inter-thread/inter-process communication
#include <uORB/topics/test_motor.h> // uORB for test_motor

__EXPORT int hg_motortest_main(int argc, char *argv[]); // export main for starting in other thread

int hg_motortest_main(int argc, char *argv[])
{
    PX4_INFO("Hello Hovergames MOTORTEST");

    struct test_motor_s test_motor;                                               // structur with test_motor paramters
    memset(&test_motor, 0, sizeof(test_motor));                                   // fill the structure with 0
    orb_advert_t test_motor_pub = orb_advertise(ORB_ID(test_motor), &test_motor); // advertise structure for ORB_ID

    test_motor.motor_number = 4;
    test_motor.value = 0.2;

    orb_publish(ORB_ID(test_motor), test_motor_pub, &test_motor); // publish the message to uORB service

    PX4_INFO("Hovergames MOTORTEST exit"); // print in console 

    return 0; // return of main function
}
