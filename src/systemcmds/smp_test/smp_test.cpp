/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/smp_out.h>

#include <string.h>
#include <unistd.h>

static void usage()
{
	PX4_INFO("Usage: smp_test send [-m msgid] [-p payload] [-s sysid] [-c compid] [-q seq] [-n count] [-i ms]");
}

extern "C" __EXPORT int smp_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (strcmp(argv[1], "send") != 0) {
		usage();
		return 1;
	}

	uint16_t msgid = 1;
	uint8_t sysid = 1;
	uint8_t compid = 1;
	uint8_t seq = 0;
	unsigned count = 1;
	unsigned interval_ms = 1000;
	const char *payload_str = "HB";

	int myoptind = 2;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:p:s:c:q:n:i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			msgid = (uint16_t)strtoul(myoptarg, nullptr, 10);
			break;

		case 'p':
			payload_str = myoptarg;
			break;

		case 's':
			sysid = (uint8_t)strtoul(myoptarg, nullptr, 10);
			break;

		case 'c':
			compid = (uint8_t)strtoul(myoptarg, nullptr, 10);
			break;

		case 'q':
			seq = (uint8_t)strtoul(myoptarg, nullptr, 10);
			break;

		case 'n':
			count = (unsigned)strtoul(myoptarg, nullptr, 10);
			break;

		case 'i':
			interval_ms = (unsigned)strtoul(myoptarg, nullptr, 10);
			break;

		default:
			usage();
			return 1;
		}
	}

	uORB::Publication<smp_out_s> pub{ORB_ID(smp_out)};
	smp_out_s tmp{};
	const size_t payload_len = strnlen(payload_str, sizeof(tmp.payload));

	for (unsigned i = 0; i < count; i++) {
		smp_out_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.sysid = sysid;
		msg.compid = compid;
		msg.seq = seq++;
		msg.msg_id = msgid;
		msg.payload_len = (uint16_t)payload_len;
		memcpy(msg.payload, payload_str, payload_len);
		msg.crc = 0;
		pub.publish(msg);

		if (interval_ms > 0 && i + 1 < count) {
			usleep(interval_ms * 1000);
		}
	}

	return 0;
}

