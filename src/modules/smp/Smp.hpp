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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param.h>

#if defined(__PX4_POSIX)
#include <netinet/in.h>
#endif

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/smp_in.h>
#include <uORB/topics/smp_out.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_gps.h>

class Smp : public ModuleBase<Smp>, public ModuleParams
{
public:
	Smp(const char *device, unsigned baudrate);
	~Smp() override;

	static int task_spawn(int argc, char *argv[]);
	static Smp *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	enum class LinkMode {
		Serial,
		TcpServer,
		TcpClient,
		UdpServer,
		UdpClient
	};

	static constexpr uint8_t kSync1 = 0xA5;
	static constexpr size_t kMaxPayload = 31;
	static constexpr size_t kMinFrame = 1 + 1 + 1 + 1 + 1 + 1 + 2;
	static constexpr size_t kMaxFrame = 1 + 1 + 1 + 1 + 1 + 1 + kMaxPayload + 2;
	static constexpr uint8_t kMsgHeartbeat = 0x01;
	static constexpr uint8_t kMsgVehData = 0x02;
	static constexpr uint8_t kMsgSysStatus = 0x10;
	static constexpr uint8_t kMsgBatteryStatus = 0x11;
	static constexpr uint8_t kMsgGpsFix = 0x20;
	static constexpr uint8_t kMsgGlobalPos = 0x21;
	static constexpr uint8_t kMsgAttitude = 0x30;
	static constexpr uint8_t kMsgHeading = 0x31;
	static constexpr uint8_t kMsgAirdata = 0x40;
	static constexpr uint8_t kMsgCommandAck = 0x90;
	static constexpr uint8_t kMsgCommandDo = 0x91;
	static constexpr uint8_t kMsgCommandShort = 0x92;
	static constexpr uint8_t kMsgCommandLong = 0x93;
	static constexpr uint8_t kMsgParamSet = 0xB0;
	static constexpr uint8_t kMsgParamGet = 0xB1;

	char _device[64] {};
	unsigned _baudrate{115200};
	int _fd{-1};
	int _listen_fd{-1};
	LinkMode _link_mode{LinkMode::Serial};
	char _tcp_host[64] {};
	uint16_t _tcp_port{0};
	uint8_t _sysid{1};
	uint8_t _compid{1};
	uint8_t _tx_seq{0};

	uint8_t _rx_buf[256] {};
	size_t _rx_len{0};
	uint8_t _last_cmd_seq[256] {};
#if defined(__PX4_POSIX)
	sockaddr_in _udp_peer{};
	bool _udp_peer_valid{false};
#endif

	uORB::Publication<smp_in_s> _in_pub{ORB_ID(smp_in)};
	uORB::Subscription _out_sub{ORB_ID(smp_out)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _cpuload_sub{ORB_ID(cpuload)};
	uORB::Subscription _health_report_sub{ORB_ID(health_report)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_ang_vel_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _gps_sub{ORB_ID(vehicle_gps_position)};

	vehicle_status_s _vehicle_status{};
	battery_status_s _battery_status{};
	cpuload_s _cpuload{};
	health_report_s _health_report{};
	airspeed_s _airspeed{};
	vehicle_thrust_setpoint_s _thrust_sp{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_angular_velocity_s _vehicle_ang_vel{};
	vehicle_local_position_s _vehicle_local_pos{};
	vehicle_global_position_s _vehicle_global_pos{};
	sensor_gps_s _gps{};
	bool _have_vehicle_status{false};
	bool _have_battery_status{false};
	bool _have_cpuload{false};
	bool _have_health_report{false};
	bool _have_airspeed{false};
	bool _have_thrust_sp{false};
	bool _have_vehicle_attitude{false};
	bool _have_vehicle_ang_vel{false};
	bool _have_vehicle_local_pos{false};
	bool _have_vehicle_global_pos{false};
	bool _have_gps{false};
	hrt_abstime _last_hb_us{0};
	hrt_abstime _last_veh_us{0};
	hrt_abstime _last_sys_us{0};
	hrt_abstime _last_batt_us{0};
	hrt_abstime _last_att_us{0};
	hrt_abstime _last_global_us{0};
	hrt_abstime _last_gps_us{0};
	hrt_abstime _last_heading_us{0};
	hrt_abstime _last_air_us{0};

	bool _echo_enabled{false};
	bool _local_ack{false};
	uint8_t _ack_msgid{0xFF};
	uint8_t _ack_payload_len{3};
	uint8_t _ack_payload[16]{'A', 'C', 'K'};

	param_t _param_smp_max_speed{PARAM_INVALID};
	param_t _param_smp_max_alt{PARAM_INVALID};
	param_t _param_smp_endur{PARAM_INVALID};
	param_t _param_smp_payload{PARAM_INVALID};
	param_t _param_smp_fw_ver{PARAM_INVALID};

	bool open_port();
	bool open_tcp();
	bool open_udp();
	void close_port();
	void close_client();
	int set_baudrate(int fd, unsigned baud);
	void handle_rx();
	void handle_tx();
	void process_rx_buffer();
	void send_echo(uint8_t sysid, uint8_t compid, uint8_t seq);
	void handle_telemetry();
	void send_msg(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len);
	ssize_t send_bytes(const uint8_t *buf, size_t len);
	static void append_u16(uint8_t *buf, size_t &offset, uint16_t value);
	static void append_i16(uint8_t *buf, size_t &offset, int16_t value);
	static void append_u32(uint8_t *buf, size_t &offset, uint32_t value);
	static void append_i32(uint8_t *buf, size_t &offset, int32_t value);
	static void append_u64(uint8_t *buf, size_t &offset, uint64_t value);
	static void append_u8(uint8_t *buf, size_t &offset, uint8_t value);
	static void append_f32(uint8_t *buf, size_t &offset, float value);
	static float read_f32(const uint8_t *buf);
	static double read_f64(const uint8_t *buf);
	uint16_t crc16_ccitt(const uint8_t *data, size_t len) const;
	static bool parse_device(const char *device, LinkMode &mode, char *host, size_t host_len, uint16_t &port);
	static bool set_nonblocking(int fd);
};
