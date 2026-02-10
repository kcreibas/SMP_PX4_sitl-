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

#include "Smp.hpp"

#include <errno.h>
#include <math.h>
#include <cmath>
#include <cstdint>
#include <poll.h>
#include <string.h>
#include <termios.h>

#if defined(__PX4_POSIX)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#endif

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <commander/px4_custom_mode.h>

static inline bool is_would_block(int err)
{
#if defined(EWOULDBLOCK) && (EWOULDBLOCK != EAGAIN)
	return err == EAGAIN || err == EWOULDBLOCK;
#else
	return err == EAGAIN;
#endif
}

static constexpr uint8_t kSmpCmdArmDisarm = 1;
static constexpr uint8_t kSmpCmdSetMode = 2;
static constexpr uint8_t kSmpCmdTakeoff = 3;
static constexpr uint8_t kSmpCmdGoto = 50;
static constexpr uint8_t kModeCustomEnabled = 1; // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

static inline void map_smp_mode_to_px4(uint8_t smp_mode, uint8_t &main_mode, uint8_t &sub_mode)
{
	main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
	sub_mode = 0;

	switch (smp_mode) {
	case 0: // STABILIZE
		main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;
	case 1: // ACRO
		main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;
	case 2: // ALT_HOLD
		main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;
	case 3: // AUTO
		main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;
	case 4: // GUIDED
		main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;
	case 5: // LOITER
		main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_POSCTL;
		break;
	case 6: // RTL
		main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;
	case 7: // CIRCLE (mapped to AUTO_LOITER)
		main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;
	case 8: // LAND
		main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;
	default:
		main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		sub_mode = 0;
		break;
	}
}

static inline uint8_t map_mav_cmd_to_smp(uint16_t mav_cmd)
{
	switch (mav_cmd) {
	case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:
		return kSmpCmdArmDisarm;
	case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE:
		return kSmpCmdSetMode;
	case vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF:
		return kSmpCmdTakeoff;
	case vehicle_command_s::VEHICLE_CMD_DO_REPOSITION:
		return kSmpCmdGoto;
	default:
		return (uint8_t)(mav_cmd & 0xFF);
	}
}

// Construct SMP module with device path and baudrate.
// 디바이스 경로와 보드레이트로 SMP 모듈을 생성합니다.
Smp::Smp(const char *device, unsigned baudrate)
	: ModuleParams(nullptr),
	  _baudrate(baudrate)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';

	_param_smp_max_speed = param_find("SMP_MAX_SPD");
	_param_smp_max_alt = param_find("SMP_MAX_ALT");
	_param_smp_endur = param_find("SMP_ENDUR");
	_param_smp_payload = param_find("SMP_PAYLOAD");
	_param_smp_fw_ver = param_find("SMP_FW_VER");
}

// Ensure serial port is closed on teardown.
// 종료 시 시리얼 포트를 닫습니다.
Smp::~Smp()
{
	close_port();
}

// Spawn the SMP task.
// SMP 태스크를 생성합니다.
int Smp::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("smp",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(2000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

// Parse CLI args and create the SMP instance.
// CLI 인자를 파싱하고 SMP 인스턴스를 생성합니다.
Smp *Smp::instantiate(int argc, char *argv[])
{
	const char *device = "/dev/ttyACM0";
	unsigned baudrate = 115200;
	bool error_flag = false;
	bool echo_enabled = false;
	uint8_t ack_msgid = 0xFF;
	const char *ack_payload = "ACK";
	uint8_t sysid = 1;
	uint8_t compid = 1;
	bool local_ack = false;
	LinkMode link_mode = LinkMode::Serial;
	char tcp_host[64] {};
	uint16_t tcp_port = 0;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:ea:p:s:c:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			baudrate = strtoul(myoptarg, nullptr, 10);
			break;

		case 'e':
			echo_enabled = true;
			break;

		case 'a': {
			const unsigned long tmp = strtoul(myoptarg, nullptr, 0);
			ack_msgid = (uint8_t)(tmp & 0xFF);
			break;
		}

		case 'p':
			ack_payload = myoptarg;
			break;

		case 's':
			sysid = (uint8_t)strtoul(myoptarg, nullptr, 10);
			break;

		case 'c':
			compid = (uint8_t)strtoul(myoptarg, nullptr, 10);
			break;

		default:
			error_flag = true;
			break;
		}
	}

	if (!parse_device(device, link_mode, tcp_host, sizeof(tcp_host), tcp_port)) {
		PX4_ERR("invalid device string: %s", device);
		error_flag = true;
	}

#if defined(__PX4_POSIX)
	if (const char *env_local_ack = getenv("PX4_SMP_LOCAL_ACK")) {
		local_ack = (atoi(env_local_ack) != 0);
	}
#endif

	if (error_flag) {
		return nullptr;
	}

	Smp *instance = new Smp(device, baudrate);

	if (!instance) {
		PX4_ERR("alloc failed");
	}

	instance->_echo_enabled = echo_enabled;
	instance->_ack_msgid = ack_msgid;
	instance->_ack_payload_len = (uint8_t)strnlen(ack_payload, sizeof(instance->_ack_payload));
	memcpy(instance->_ack_payload, ack_payload, instance->_ack_payload_len);
	instance->_sysid = sysid;
	instance->_compid = compid;
	instance->_local_ack = local_ack;
	instance->_link_mode = link_mode;
	if (link_mode != LinkMode::Serial) {
		strncpy(instance->_tcp_host, tcp_host, sizeof(instance->_tcp_host) - 1);
		instance->_tcp_host[sizeof(instance->_tcp_host) - 1] = '\0';
		instance->_tcp_port = tcp_port;
	}
	return instance;
}

// No custom commands; always return usage.
// 별도 커스텀 명령이 없으므로 사용법을 출력합니다.
int Smp::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// Print module usage and protocol summary.
// 모듈 사용법과 프로토콜 요약을 출력합니다.
int Smp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
UART bridge for a simple custom binary protocol (SMP v1).
Device: /dev/ttyXXX (serial) or tcp://:port (server) or tcp://host:port (client, POSIX).
        udp://:port (server) or udp://host:port (client, POSIX).
Frame: sync(0xA5), sysid, comid, seq, msgid(u8), len(u8), payload, crc16.
COMMAND_DO(0x91) payload: u8 cmd + i32 p1 + u8 target_sys + u8 target_comp.
COMMAND_SHORT(0x92) payload: u8 cmd + i32 p1..p4 + u8 target_sys + u8 target_comp.
COMMAND_LONG(0x93) payload: u8 cmd + i32 p1..p7 + u8 target_sys + u8 target_comp.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("smp", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyACM0", "<file:dev>", "Serial device path", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 9600, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Echo ACK frame on valid receive", true);
	PRINT_MODULE_USAGE_PARAM_INT('a', 255, 0, 255, "ACK msgid (when -e set)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', "ACK", "<string>", "ACK payload (when -e set)", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', 1, 0, 255, "Sysid", true);
	PRINT_MODULE_USAGE_PARAM_INT('c', 1, 0, 255, "Compid", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// Main loop: open port, poll RX, then TX and telemetry.
// 메인 루프: 포트 오픈 → RX 폴링 → TX/텔레메트리 처리.
void Smp::run()
{
	while (!should_exit()) {
		if (!open_port()) {
			px4_sleep(1);
			continue;
		}

		px4_pollfd_struct_t fds {};
		fds.fd = _fd;
		fds.events = POLLIN | POLLERR | POLLHUP;

		int pret = px4_poll(&fds, 1, 50);

		if (pret > 0 && (fds.revents & (POLLIN | POLLERR | POLLHUP))) {
			handle_rx();
		} else if (_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient ||
			   _link_mode == LinkMode::UdpServer || _link_mode == LinkMode::UdpClient) {
			handle_rx();
		}

		handle_tx();
		handle_telemetry();
	}

	close_port();
}

// Open and configure the serial port.
// 시리얼 포트를 열고 설정합니다. @@@
bool Smp::open_port()
{
	if (_fd >= 0) {
		return true;
	}

	if (_link_mode != LinkMode::Serial) {
		if (_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient) {
			return open_tcp();
		}
		return open_udp();
	}

	_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed: %d", _device, errno);
		return false;
	}

	if (set_baudrate(_fd, _baudrate) != 0) {
		PX4_ERR("baudrate setup failed");
		close_port();
		return false;
	}

	return true;
}

bool Smp::open_tcp()
{
#if !defined(__PX4_POSIX)
	PX4_ERR("tcp not supported on this platform");
	return false;
#else
	if (_link_mode == LinkMode::TcpServer) {
		if (_listen_fd < 0) {
			_listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
			if (_listen_fd < 0) {
				PX4_ERR("tcp socket failed: %d", errno);
				return false;
			}

			int yes = 1;
			(void)setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

			sockaddr_in addr {};
			addr.sin_family = AF_INET;
			addr.sin_port = htons(_tcp_port);
			addr.sin_addr.s_addr = htonl(INADDR_ANY);

			if (::bind(_listen_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
				PX4_ERR("tcp bind failed: %d", errno);
				close_port();
				return false;
			}

			if (::listen(_listen_fd, 1) < 0) {
				PX4_ERR("tcp listen failed: %d", errno);
				close_port();
				return false;
			}

			set_nonblocking(_listen_fd);
			PX4_INFO("SMP TCP server listening on :%u", (unsigned)_tcp_port);
		}

		sockaddr_in client {};
		socklen_t client_len = sizeof(client);
		int cfd = ::accept(_listen_fd, (sockaddr *)&client, &client_len);

		if (cfd < 0) {
			if (is_would_block(errno)) {
				return false;
			}
			PX4_ERR("tcp accept failed: %d", errno);
			return false;
		}

		set_nonblocking(cfd);
		_fd = cfd;
		PX4_INFO("SMP TCP client connected");
		return true;
	}

	// Tcp client
	int fd = ::socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		PX4_ERR("tcp socket failed: %d", errno);
		return false;
	}

	sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(_tcp_port);
	if (::inet_aton(_tcp_host, &addr.sin_addr) == 0) {
		PX4_ERR("invalid tcp host: %s", _tcp_host);
		px4_close(fd);
		return false;
	}

	if (::connect(fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("tcp connect failed: %d", errno);
		px4_close(fd);
		return false;
	}

	set_nonblocking(fd);
	_fd = fd;
	PX4_INFO("SMP TCP connected to %s:%u", _tcp_host, (unsigned)_tcp_port);
	return true;
#endif
}

bool Smp::open_udp()
{
#if !defined(__PX4_POSIX)
	PX4_ERR("udp not supported on this platform");
	return false;
#else
	if (_link_mode == LinkMode::UdpServer) {
		if (_fd < 0) {
			int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
			if (fd < 0) {
				PX4_ERR("udp socket failed: %d", errno);
				return false;
			}

			int yes = 1;
			(void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

			sockaddr_in addr {};
			addr.sin_family = AF_INET;
			addr.sin_port = htons(_tcp_port);
			addr.sin_addr.s_addr = htonl(INADDR_ANY);

			if (::bind(fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
				PX4_ERR("udp bind failed: %d", errno);
				px4_close(fd);
				return false;
			}

			set_nonblocking(fd);
			_fd = fd;
			_udp_peer_valid = false;
			PX4_INFO("SMP UDP server listening on :%u", (unsigned)_tcp_port);
		}
		return true;
	}

	// Udp client
	int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		PX4_ERR("udp socket failed: %d", errno);
		return false;
	}

	sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(_tcp_port);
	if (::inet_aton(_tcp_host, &addr.sin_addr) == 0) {
		PX4_ERR("invalid udp host: %s", _tcp_host);
		px4_close(fd);
		return false;
	}

	if (::connect(fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("udp connect failed: %d", errno);
		px4_close(fd);
		return false;
	}

	set_nonblocking(fd);
	_fd = fd;
	PX4_INFO("SMP UDP connected to %s:%u", _tcp_host, (unsigned)_tcp_port);
	return true;
#endif
}

// Close serial port if open.
// 열려 있는 시리얼 포트를 닫습니다.
void Smp::close_port()
{
	close_client();
	if (_listen_fd >= 0) {
		px4_close(_listen_fd);
		_listen_fd = -1;
	}
#if defined(__PX4_POSIX)
	_udp_peer_valid = false;
#endif
}

void Smp::close_client()
{
	if (_fd >= 0) {
		px4_close(_fd);
		_fd = -1;
	}
}

// Apply baudrate and raw UART settings.
// 보드레이트와 UART 로우 설정을 적용합니다. @@@@
int Smp::set_baudrate(int fd, unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
#ifndef B460800
#define B460800 460800
#endif
	case 460800: speed = B460800; break;
#ifndef B921600
#define B921600 921600
#endif
	case 921600: speed = B921600; break;
	default:
		PX4_ERR("unknown baudrate: %u", baud);
		return -EINVAL;
	}

	struct termios uart_config;
	int termios_state = tcgetattr(fd, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("tcgetattr failed: %d", termios_state);
		return -1;
	}

	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	uart_config.c_oflag = 0;
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	uart_config.c_cflag |= (CLOCAL | CREAD);
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("cfsetispeed/cfsetospeed failed");
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed");
		return -1;
	}

	return 0;
}

// Transport RX:
// 포트(TCP/UDP/Serial)에서 바이트를 읽어 _rx_buf에 누적합니다.
// 파싱/검증은 process_rx_buffer()에서 수행합니다.
void Smp::handle_rx()
{
	uint8_t buf[64];
	int nread = 0;
	static uint64_t last_rx_log_us = 0;

#if defined(__PX4_POSIX)
	if (_link_mode == LinkMode::UdpServer) {
		sockaddr_in peer {};
		socklen_t peer_len = sizeof(peer);
		nread = ::recvfrom(_fd, buf, sizeof(buf), 0, (sockaddr *)&peer, &peer_len);
		if (nread > 0) {
			_udp_peer = peer;
			_udp_peer_valid = true;
		}
	} else
#endif
	{
		nread = ::read(_fd, buf, sizeof(buf));
	}

	if (nread <= 0) {
		if (_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient) {
			if (nread == 0 || !is_would_block(errno)) {
				PX4_WARN("tcp rx closed: %d", errno);
				close_client();
			}
			return;
		}

		if (_link_mode == LinkMode::UdpServer || _link_mode == LinkMode::UdpClient) {
			if (nread < 0 && !is_would_block(errno)) {
				PX4_WARN("udp rx failed: %d", errno);
			}
			return;
		}
		return;
	}

	const uint64_t now = hrt_absolute_time();
	if (now - last_rx_log_us > 1000000) {
		const uint8_t b0 = buf[0];
		const uint8_t b1 = (nread > 1) ? buf[1] : 0;
		const uint8_t b2 = (nread > 2) ? buf[2] : 0;
		const uint8_t b3 = (nread > 3) ? buf[3] : 0;
		PX4_INFO("SMP RX bytes: %d first=%02x %02x %02x %02x", nread, b0, b1, b2, b3);
		last_rx_log_us = now;
	}

	if (_rx_len + (size_t)nread > sizeof(_rx_buf)) {
		_rx_len = 0;
	}

	memcpy(&_rx_buf[_rx_len], buf, nread);
	_rx_len += nread;
	process_rx_buffer();
}

// RX 파이프라인:
// 1) sync(0xA5) 위치 탐색 → 2) 길이 확인 → 3) CRC16 검증
// 4) smp_in(uORB) 게시 → 5) COMMAND_*를 vehicle_command로 변환
// 새 RX 메시지 추가는 이 함수에 분기 추가 (msg_id/len 기준).
void Smp::process_rx_buffer()
{
	// SMP RX handlers (conceptual FW function names):
	// - SMP_COMMAND_DO()   : msgid 0x91, len 7
	// - SMP_COMMAND_SHORT(): msgid 0x92, len 19
	// - SMP_COMMAND_LONG() : msgid 0x93, len 31
	// - SMP_PARAM_SET/GET  : not implemented (reserved)
	static uint64_t last_rx_log_us = 0;
	static uint64_t last_crc_log_us = 0;
	static uint64_t last_len_log_us = 0;
	while (_rx_len >= kMinFrame) {
		size_t sync_pos = 0;
		bool found = false;

		for (; sync_pos < _rx_len; sync_pos++) {
			if (_rx_buf[sync_pos] == kSync1) {
				found = true;
				break;
			}
		}

		if (!found) {
			_rx_len = 0;
			return;
		}

		if (sync_pos > 0) {
			memmove(_rx_buf, &_rx_buf[sync_pos], _rx_len - sync_pos);
			_rx_len -= sync_pos;
		}

		if (_rx_len < kMinFrame) {
			return;
		}

		uint8_t sysid = _rx_buf[1];
		uint8_t compid = _rx_buf[2];
		uint8_t seq = _rx_buf[3];
		uint8_t msg_id = _rx_buf[4];
		uint8_t payload_len = _rx_buf[5];

		if (payload_len > kMaxPayload) {
			const uint64_t now = hrt_absolute_time();
			if (now - last_len_log_us > 1000000) {
				PX4_WARN("SMP invalid len: %u (sync=0x%02x)", payload_len, _rx_buf[0]);
				last_len_log_us = now;
			}
			memmove(_rx_buf, &_rx_buf[1], _rx_len - 1);
			_rx_len -= 1;
			continue;
		}

		size_t frame_len = 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2;

		if (_rx_len < frame_len) {
			return;
		}

		const uint8_t *payload = &_rx_buf[6];
		uint16_t crc_rx = (uint16_t)_rx_buf[6 + payload_len] |
				  ((uint16_t)_rx_buf[6 + payload_len + 1] << 8);

		uint8_t crc_buf[5 + kMaxPayload] {};
		crc_buf[0] = sysid;
		crc_buf[1] = compid;
		crc_buf[2] = seq;
		crc_buf[3] = msg_id;
		crc_buf[4] = payload_len;
		memcpy(&crc_buf[5], payload, payload_len);

		uint16_t crc_calc = crc16_ccitt(crc_buf, 5 + payload_len);

		if (crc_calc == crc_rx) {
			smp_in_s msg{};
			msg.timestamp = hrt_absolute_time();
			msg.sysid = sysid;
			msg.compid = compid;
			msg.seq = seq;
			msg.msg_id = msg_id;
			msg.payload_len = payload_len;
			memcpy(msg.payload, payload, payload_len);
			msg.crc = crc_rx;
			_in_pub.publish(msg);
			const uint64_t now = hrt_absolute_time();
			if (now - last_rx_log_us > 1000000) {
				PX4_INFO("SMP RX: msg=0x%02x len=%u", msg_id, payload_len);
				last_rx_log_us = now;
			}

			if (_echo_enabled) {
				send_echo(sysid, compid, seq);
			}
			// COMMAND_* payload 구조 요약:
			// - COMMAND_DO   (len=7):  [cmd_id][p1(int32)][tgt_sys][tgt_comp]
			// - COMMAND_SHORT(len=19): [cmd_id][p1..p4(int32)][tgt_sys][tgt_comp]
			// - COMMAND_LONG (len=31): [cmd_id][p1..p7(int32)][tgt_sys][tgt_comp]
			if ((msg_id == kMsgCommandDo && payload_len == 7) ||
			    (msg_id == kMsgCommandShort && payload_len == 19) ||
			    (msg_id == kMsgCommandLong && payload_len == 31)) {
				const uint8_t smp_cmd = payload[0];
				_last_cmd_seq[smp_cmd] = seq;

				vehicle_command_s cmd{};
				cmd.timestamp = hrt_absolute_time();
				cmd.command = smp_cmd;

				cmd.param1 = NAN;
				cmd.param2 = NAN;
				cmd.param3 = NAN;
				cmd.param4 = NAN;
				cmd.param5 = NAN;
				cmd.param6 = NAN;
				cmd.param7 = NAN;

				int32_t p1_i32 = 0;
				int32_t p2_i32 = 0;
				int32_t p3_i32 = 0;
				int32_t p4_i32 = 0;
				int32_t p5_i32 = 0;
				int32_t p6_i32 = 0;
				int32_t p7_i32 = 0;
				bool have_p1 = false;
				bool have_p2 = false;
				bool have_p3 = false;

				uint8_t target_system = 0;
				uint8_t target_component = 0;

				if (msg_id == kMsgCommandDo) {
					p1_i32 = read_i32(&payload[1]);
					cmd.param1 = (float)p1_i32;
					have_p1 = true;
					target_system = payload[5];
					target_component = payload[6];
				} else if (msg_id == kMsgCommandShort) {
					p1_i32 = read_i32(&payload[1]);
					p2_i32 = read_i32(&payload[5]);
					p3_i32 = read_i32(&payload[9]);
					p4_i32 = read_i32(&payload[13]);
					cmd.param1 = (float)p1_i32;
					cmd.param2 = (float)p2_i32;
					cmd.param3 = (float)p3_i32;
					cmd.param4 = (float)p4_i32;
					have_p1 = true;
					have_p2 = true;
					have_p3 = true;
					target_system = payload[17];
					target_component = payload[18];
				} else if (msg_id == kMsgCommandLong) {
					p1_i32 = read_i32(&payload[1]);
					p2_i32 = read_i32(&payload[5]);
					p3_i32 = read_i32(&payload[9]);
					p4_i32 = read_i32(&payload[13]);
					p5_i32 = read_i32(&payload[17]);
					p6_i32 = read_i32(&payload[21]);
					p7_i32 = read_i32(&payload[25]);
					cmd.param1 = (float)p1_i32;
					cmd.param2 = (float)p2_i32;
					cmd.param3 = (float)p3_i32;
					cmd.param4 = (float)p4_i32;
					cmd.param5 = (float)p5_i32;
					cmd.param6 = (float)p6_i32;
					cmd.param7 = (float)p7_i32;
					have_p1 = true;
					have_p2 = true;
					have_p3 = true;
					target_system = payload[29];
					target_component = payload[30];
				}

				if (smp_cmd == kSmpCmdArmDisarm) {
					int32_t arm_v = have_p1 ? p1_i32 : 0;
					if (arm_v < 0) {
						arm_v = 0;
					}

					const bool arm = (arm_v & 0x01) != 0;
					const bool force = (arm_v & 0x02) != 0;

					cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
					cmd.param1 = arm ? 1.0f : 0.0f;
					cmd.param2 = force ? 21196.0f : 0.0f;
					cmd.param3 = NAN;
					cmd.param4 = NAN;
					cmd.param5 = NAN;
					cmd.param6 = NAN;
					cmd.param7 = NAN;
				} else if (smp_cmd == kSmpCmdSetMode) {
					int mode_i = (int)lroundf(cmd.param1);
					if (mode_i < 0) {
						mode_i = 0;
					} else if (mode_i > 8) {
						mode_i = 8;
					}

					uint8_t main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
					uint8_t sub_mode = 0;
					map_smp_mode_to_px4((uint8_t)mode_i, main_mode, sub_mode);

					cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
					cmd.param1 = (float)kModeCustomEnabled;
					cmd.param2 = (float)main_mode;
					cmd.param3 = (float)sub_mode;
					cmd.param4 = NAN;
					cmd.param5 = NAN;
					cmd.param6 = NAN;
					cmd.param7 = NAN;
				} else if (smp_cmd == kSmpCmdTakeoff) {
					const float rel_alt_m = have_p1 ? (float)p1_i32 : NAN;
					float abs_alt_m = rel_alt_m;
					if (_have_vehicle_global_pos && std::isfinite(_vehicle_global_pos.alt) && std::isfinite(rel_alt_m)) {
						abs_alt_m = (float)_vehicle_global_pos.alt + rel_alt_m;
					}

					cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF;
					cmd.param1 = NAN;
					cmd.param2 = NAN;
					cmd.param3 = NAN;
					cmd.param4 = NAN;
					cmd.param5 = NAN;
					cmd.param6 = NAN;
					cmd.param7 = abs_alt_m;
				} else if (smp_cmd == kSmpCmdGoto) {
					const float lat = have_p1 ? (float)((double)p1_i32 / 1e7) : NAN;
					const float lon = have_p2 ? (float)((double)p2_i32 / 1e7) : NAN;
					const float alt = have_p3 ? (float)((double)p3_i32 / 1000.0) : NAN;

					cmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
					cmd.param1 = NAN;
					cmd.param2 = 1.0f; // change_mode_flags bit0=1 (request mode switch)
					cmd.param3 = NAN;
					cmd.param4 = NAN; // yaw unused
					cmd.param5 = lat;
					cmd.param6 = lon;
					cmd.param7 = alt;
				}

				cmd.target_system = target_system;
				cmd.target_component = target_component;
				cmd.source_system = sysid;
				cmd.source_component = compid;
				cmd.confirmation = 0;
				cmd.from_external = true;
				_vehicle_command_pub.publish(cmd);

				// 로컬 ACK: SMP 자체에서 즉시 응답(선택 사항).
				// PX4 내부 처리 결과와는 별개일 수 있습니다.
				if (_local_ack) {
					uint8_t ack_payload[3] {};
					size_t ack_off = 0;
					append_u8(ack_payload, ack_off, smp_cmd);
					append_u8(ack_payload, ack_off, 0); // MAV_RESULT_ACCEPTED
					append_u8(ack_payload, ack_off, seq);
					send_msg(kMsgCommandAck, ack_payload, (uint8_t)ack_off);
				}
			}
		} else {
			const uint64_t now = hrt_absolute_time();
			if (now - last_crc_log_us > 1000000) {
				PX4_WARN("SMP CRC mismatch: msg=0x%02x len=%u calc=0x%04x rx=0x%04x",
					 msg_id, payload_len, crc_calc, crc_rx);
				last_crc_log_us = now;
			}
		}
		memmove(_rx_buf, &_rx_buf[frame_len], _rx_len - frame_len);
		_rx_len -= frame_len;
	}
}

// smp_out로 들어온 프레임을 실제 링크로 전송합니다.
// 외부 모듈이 SMP 프레임을 직접 만들어 보낼 때 사용하는 TX 경로입니다.
void Smp::handle_tx()
{
	if (_fd < 0) {
		return;
	}

	smp_out_s out{};

	if (!_out_sub.update(&out)) {
		return;
	}

	if (out.payload_len > kMaxPayload) {
		PX4_WARN("payload too large: %u", out.payload_len);
		return;
	}

	uint8_t frame[kMaxFrame] {};
	frame[0] = kSync1;
	frame[1] = out.sysid;
	frame[2] = out.compid;
	frame[3] = out.seq;
	frame[4] = out.msg_id;
	frame[5] = out.payload_len;
	memcpy(&frame[6], out.payload, out.payload_len);

	uint8_t crc_buf[5 + kMaxPayload] {};
	crc_buf[0] = frame[1];
	crc_buf[1] = frame[2];
	crc_buf[2] = frame[3];
	crc_buf[3] = frame[4];
	crc_buf[4] = frame[5];
	memcpy(&crc_buf[5], out.payload, out.payload_len);

	uint16_t crc = crc16_ccitt(crc_buf, 5 + out.payload_len);
	frame[6 + out.payload_len] = (uint8_t)(crc & 0xFF);
	frame[6 + out.payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

	ssize_t nwritten = send_bytes(frame, 1 + 1 + 1 + 1 + 1 + 1 + out.payload_len + 2);

	if (nwritten < 0) {
		if ((_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient) && !is_would_block(errno)) {
			PX4_WARN("tcp tx failed: %d", errno);
			close_client();
		} else if (_link_mode == LinkMode::UdpServer || _link_mode == LinkMode::UdpClient) {
			if (!is_would_block(errno)) {
				PX4_WARN("udp tx failed: %d", errno);
			}
		} else if (_link_mode == LinkMode::Serial) {
			PX4_ERR("write failed: %d", errno);
		}
	}
}

// Send optional ACK/echo frame for a valid RX frame.
// 유효한 RX 프레임에 대해 ACK/에코 프레임을 보냅니다.
void Smp::send_echo(uint8_t sysid, uint8_t compid, uint8_t seq)
{
	static uint64_t last_echo_log_us = 0;
	if (_fd < 0) {
		return;
	}

	const uint8_t payload_len = _ack_payload_len;
	if (payload_len > kMaxPayload) {
		return;
	}
	const uint64_t now = hrt_absolute_time();
	if (now - last_echo_log_us > 1000000) {
		PX4_INFO("SMP ECHO: msg=0x%02x len=%u", _ack_msgid, (unsigned)payload_len);
		last_echo_log_us = now;
	}

	uint8_t frame[kMaxFrame] {};
	frame[0] = kSync1;
	frame[1] = sysid;
	frame[2] = compid;
	frame[3] = seq;
	frame[4] = _ack_msgid;
	frame[5] = payload_len;
	memcpy(&frame[6], _ack_payload, payload_len);

	uint8_t crc_buf[5 + kMaxPayload] {};
	crc_buf[0] = frame[1];
	crc_buf[1] = frame[2];
	crc_buf[2] = frame[3];
	crc_buf[3] = frame[4];
	crc_buf[4] = frame[5];
	memcpy(&crc_buf[5], _ack_payload, payload_len);

	uint16_t crc = crc16_ccitt(crc_buf, 5 + payload_len);
	frame[6 + payload_len] = (uint8_t)(crc & 0xFF);
	frame[6 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

	ssize_t nwritten = send_bytes(frame, 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2);

	if (nwritten < 0) {
		if ((_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient) && !is_would_block(errno)) {
			PX4_WARN("tcp tx failed: %d", errno);
			close_client();
		} else if (_link_mode == LinkMode::UdpServer || _link_mode == LinkMode::UdpClient) {
			if (!is_would_block(errno)) {
				PX4_WARN("udp tx failed: %d", errno);
			}
		} else if (_link_mode == LinkMode::Serial) {
			PX4_ERR("write failed: %d", errno);
		}
	}
}

static inline uint8_t clamp_u8(int32_t v)
{
	if (v < 0) {
		return 0;
	}
	if (v > 255) {
		return 255;
	}
	return (uint8_t)v;
}

static inline uint16_t clamp_u16(int32_t v)
{
	if (v < 0) {
		return 0;
	}
	if (v > 65535) {
		return 65535;
	}
	return (uint16_t)v;
}

static inline int16_t clamp_i16(int32_t v)
{
	if (v < -32768) {
		return -32768;
	}
	if (v > 32767) {
		return 32767;
	}
	return (int16_t)v;
}

static inline int8_t clamp_i8(int32_t v)
{
	if (v < -128) {
		return -128;
	}
	if (v > 127) {
		return 127;
	}
	return (int8_t)v;
}

static inline int32_t clamp_i32(int64_t v)
{
	if (v < INT32_MIN) {
		return INT32_MIN;
	}
	if (v > INT32_MAX) {
		return INT32_MAX;
	}
	return (int32_t)v;
}

// TX 파이프라인:
// uORB 토픽을 수집해 SMP 텔레메트리 프레임을 주기적으로 전송합니다.
// 새 TX 메시지를 추가하려면 이 함수에 payload 구성 + send_msg() 호출을 추가하세요.
void Smp::handle_telemetry()
{
	// SMP TX builders (conceptual FW function names):
	// - SMP_HEARTBEAT()      : msgid 0x01
	// - SMP_VEH_DATA()       : msgid 0x02
	// - SMP_SYS_STATUS()     : msgid 0x10
	// - SMP_BATTERY_STATUS() : msgid 0x11
	// - SMP_GPS_FIX()        : msgid 0x20
	// - SMP_GLOBAL_POSITION(): msgid 0x21
	// - SMP_ATTITUDE()       : msgid 0x30
	// - SMP_HEADING()        : msgid 0x31
	// - SMP_AIRDATA()        : msgid 0x40
	// - SMP_COMMAND_ACK()    : msgid 0x90
	const hrt_abstime now = hrt_absolute_time();

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
		_have_vehicle_status = true;
	}

	if (_battery_status_sub.updated()) {
		_battery_status_sub.copy(&_battery_status);
		_have_battery_status = true;
	}

	if (_cpuload_sub.updated()) {
		_cpuload_sub.copy(&_cpuload);
		_have_cpuload = true;
	}

	if (_health_report_sub.updated()) {
		_health_report_sub.copy(&_health_report);
		_have_health_report = true;
	}

	if (_airspeed_sub.updated()) {
		_airspeed_sub.copy(&_airspeed);
		_have_airspeed = true;
	}

	if (_thrust_sp_sub.updated()) {
		_thrust_sp_sub.copy(&_thrust_sp);
		_have_thrust_sp = true;
	}

	if (_vehicle_attitude_sub.updated()) {
		_vehicle_attitude_sub.copy(&_vehicle_attitude);
		_have_vehicle_attitude = true;
	}

	if (_vehicle_ang_vel_sub.updated()) {
		_vehicle_ang_vel_sub.copy(&_vehicle_ang_vel);
		_have_vehicle_ang_vel = true;
	}

	if (_vehicle_local_pos_sub.updated()) {
		_vehicle_local_pos_sub.copy(&_vehicle_local_pos);
		_have_vehicle_local_pos = true;
	}

	if (_vehicle_global_pos_sub.updated()) {
		_vehicle_global_pos_sub.copy(&_vehicle_global_pos);
		_have_vehicle_global_pos = true;
	}

	if (_gps_sub.updated()) {
		_gps_sub.copy(&_gps);
		_have_gps = true;
	}

	vehicle_command_ack_s cmd_ack{};
	if (_vehicle_command_ack_sub.update(&cmd_ack)) {
		uint8_t payload[3] {};
		size_t offset = 0;
		const uint8_t smp_cmd = map_mav_cmd_to_smp(cmd_ack.command);
		const uint8_t seq = _last_cmd_seq[smp_cmd];
		append_u8(payload, offset, smp_cmd);
		append_u8(payload, offset, cmd_ack.result);
		append_u8(payload, offset, seq);
		send_msg(kMsgCommandAck, payload, (uint8_t)offset);
	}
//smp_command_ack() # 1115
	auto map_vehicle_type = [&]() -> uint8_t {
		switch (_vehicle_status.vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			return 1; // FIXED_WING
		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			return 2; // QUADROTOR
		default:
			return 0; // GENERIC
		}
	};

	auto map_custom_mode = [&]() -> uint8_t {
		switch (_vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		case vehicle_status_s::NAVIGATION_STATE_STAB:
			return 0; // STABILIZE
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			return 1;
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE:
			return 2;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
			return 3;
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			return 4;
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		case vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW:
			return 5; // LOITER-like (position hold)
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			return 5;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			return 6;
		case vehicle_status_s::NAVIGATION_STATE_ORBIT:
			return 7;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		case vehicle_status_s::NAVIGATION_STATE_DESCEND:
			return 8;
		default:
			return 0; // STABILIZE
		}
	};

	auto map_fix_type = [&]() -> uint8_t {
		switch (_gps.fix_type) {
		case 0:
		case sensor_gps_s::FIX_TYPE_NONE:
			return 0; // NO_FIX
		case sensor_gps_s::FIX_TYPE_2D:
			return 1; // 2D_FIX
		case sensor_gps_s::FIX_TYPE_3D:
			return 2; // 3D_FIX
		case sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL:
			return 3; // DGPS
		case sensor_gps_s::FIX_TYPE_RTK_FLOAT:
			return 4; // RTK_FLOAT
		case sensor_gps_s::FIX_TYPE_RTK_FIXED:
			return 5; // RTK_FIXED
		default:
			// Map other values (e.g. EXTRAPOLATED) to 3D_FIX by default.
			return 2;
		}
	};

	using health_component_t = events::px4::enums::health_component_t;

	auto health_ok = [&] (health_component_t comp) -> bool {
		if (!_have_health_report) {
			return false;
		}

		const uint64_t mask = (uint64_t)comp;
		if ((_health_report.health_is_present_flags & mask) == 0) {
			return false;
		}

		const uint64_t bad = _health_report.arming_check_error_flags |
				     _health_report.arming_check_warning_flags |
				     _health_report.health_error_flags |
				     _health_report.health_warning_flags;
		return (bad & mask) == 0;
	};

	if (_have_vehicle_status && (now - _last_veh_us) >= 5000000) {
		auto read_param_u16 = [&](param_t h) -> uint16_t {
			int32_t v = 0;
			if (h != PARAM_INVALID) {
				param_get(h, &v);
			}
			return clamp_u16(v);
		};

		const uint16_t max_speed = read_param_u16(_param_smp_max_speed);
		const uint16_t max_alt = read_param_u16(_param_smp_max_alt);
		const uint16_t endurance = read_param_u16(_param_smp_endur);
		const uint16_t payload_cap = read_param_u16(_param_smp_payload);
		const uint16_t fw_ver = read_param_u16(_param_smp_fw_ver);

		uint8_t payload[16] {};
		size_t offset = 0;
		append_u8(payload, offset, map_vehicle_type());
		append_u16(payload, offset, max_speed);
		append_u16(payload, offset, max_alt);
		append_u16(payload, offset, endurance);
		append_u16(payload, offset, payload_cap);
		append_u16(payload, offset, fw_ver);
		send_msg(kMsgVehData, payload, (uint8_t)offset);
		_last_veh_us = now;
	}
//smp_veh_data() #1222~1227
	if (_have_vehicle_status && (now - _last_hb_us) >= 1000000) {
		const uint8_t base_mode = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
		const uint8_t custom_mode = map_custom_mode();
		const uint8_t system_status = _vehicle_status.failsafe ? 5 : (base_mode ? 4 : 3);
		const uint8_t autopilot = 1;

		uint8_t payload[8] {};
		size_t offset = 0;
		append_u8(payload, offset, map_vehicle_type());
		append_u8(payload, offset, autopilot);
		append_u8(payload, offset, base_mode);
		append_u8(payload, offset, custom_mode);
		append_u8(payload, offset, system_status);
		send_msg(kMsgHeartbeat, payload, (uint8_t)offset);
		_last_hb_us = now;
	}
// smp_heartbeat() #1244
	if (_have_battery_status && (now - _last_sys_us) >= 1000000) {
		const float vbatt_v = std::isfinite(_battery_status.voltage_v) ? _battery_status.voltage_v : 0.0f;
		const float ibatt_a = std::isfinite(_battery_status.current_a) ? _battery_status.current_a : 0.0f;
		const uint16_t vbatt_mv = clamp_u16((int32_t)lroundf(vbatt_v * 1000.0f));
		const int16_t ibatt_ma = clamp_i16((int32_t)lroundf(ibatt_a * 1000.0f));
		const uint8_t rem = clamp_u8((int32_t)lroundf(std::isfinite(_battery_status.remaining) ?
								 (_battery_status.remaining * 100.0f) : 0.0f));
		uint8_t sens_health = 0;
		if (health_ok(health_component_t::gps)) {
			sens_health |= 0x01;
		}
		if (health_ok(health_component_t::gyro)) {
			sens_health |= 0x02;
		}
		if (health_ok(health_component_t::accel)) {
			sens_health |= 0x04;
		}
		if (health_ok(health_component_t::magnetometer)) {
			sens_health |= 0x08;
		}
		if (health_ok(health_component_t::absolute_pressure)) {
			sens_health |= 0x10;
		}
		if (health_ok(health_component_t::optical_flow)) {
			sens_health |= 0x20;
		}
		if (health_ok(health_component_t::vision_position)) {
			sens_health |= 0x40;
		}
		if (health_ok(health_component_t::distance_sensor)) {
			sens_health |= 0x80;
		}

		const uint8_t cpu_load = _have_cpuload
					 ? clamp_u8((int32_t)lroundf(_cpuload.load * 100.0f))
					 : 0;

		uint8_t payload[16] {};
		size_t offset = 0;
		append_u8(payload, offset, sens_health);
		append_u8(payload, offset, cpu_load);
		append_u16(payload, offset, vbatt_mv);
		append_i16(payload, offset, ibatt_ma);
		append_u8(payload, offset, rem);
		append_u16(payload, offset, 0);
		send_msg(kMsgSysStatus, payload, (uint8_t)offset);
		_last_sys_us = now;
	}
//smp_sys_status() #1289~1293
	if (_have_battery_status && (now - _last_batt_us) >= 1000000) {
		const float vbatt_v = std::isfinite(_battery_status.voltage_v) ? _battery_status.voltage_v : 0.0f;
		const float ibatt_a = std::isfinite(_battery_status.current_a) ? _battery_status.current_a : 0.0f;
		const uint16_t vbatt_mv = clamp_u16((int32_t)lroundf(vbatt_v * 1000.0f));
		const int16_t ibatt_ma = clamp_i16((int32_t)lroundf(ibatt_a * 1000.0f));
		const uint8_t rem = clamp_u8((int32_t)lroundf(std::isfinite(_battery_status.remaining) ?
								 (_battery_status.remaining * 100.0f) : 0.0f));
		const uint16_t time_rem = clamp_u16((int32_t)lroundf(std::isfinite(_battery_status.time_remaining_s) ?
									_battery_status.time_remaining_s : 0.0f));
		const uint8_t cell_cnt = _battery_status.cell_count;
		const int8_t temp_c = clamp_i8((int32_t)lroundf(std::isfinite(_battery_status.temperature) ?
								     _battery_status.temperature : 0.0f));

		uint8_t payload[16] {};
		size_t offset = 0;
		append_u16(payload, offset, vbatt_mv);
		append_i16(payload, offset, ibatt_ma);
		append_u8(payload, offset, rem);
		append_u16(payload, offset, time_rem);
		append_u8(payload, offset, cell_cnt);
		append_u8(payload, offset, (uint8_t)temp_c);
		send_msg(kMsgBatteryStatus, payload, (uint8_t)offset);
		_last_batt_us = now;
	}
//smp_battery_status() #1312~1318
	if (_have_gps && (now - _last_gps_us) >= 1000000) {
		const uint16_t hdop = clamp_u16((int32_t)lroundf(std::isfinite(_gps.hdop) ? _gps.hdop * 100.0f : 0.0f));
		const uint16_t vdop = clamp_u16((int32_t)lroundf(std::isfinite(_gps.vdop) ? _gps.vdop * 100.0f : 0.0f));
		const uint16_t eph = clamp_u16((int32_t)lroundf(std::isfinite(_gps.eph) ? _gps.eph * 100.0f : 0.0f));
		const uint16_t epv = clamp_u16((int32_t)lroundf(std::isfinite(_gps.epv) ? _gps.epv * 100.0f : 0.0f));

		uint8_t payload[16] {};
		size_t offset = 0;
		append_u8(payload, offset, map_fix_type());
		append_u8(payload, offset, _gps.satellites_used);
		append_u16(payload, offset, hdop);
		append_u16(payload, offset, vdop);
		append_u16(payload, offset, eph);
		append_u16(payload, offset, epv);
		send_msg(kMsgGpsFix, payload, (uint8_t)offset);
		_last_gps_us = now;
	}
//smp_gps_fix() # 1312~1318
	if (_have_vehicle_global_pos && _have_vehicle_local_pos && (now - _last_global_us) >= 200000) {
		const double lat_deg = std::isfinite(_vehicle_global_pos.lat) ? _vehicle_global_pos.lat : 0.0;
		const double lon_deg = std::isfinite(_vehicle_global_pos.lon) ? _vehicle_global_pos.lon : 0.0;
		const float alt_m = std::isfinite(_vehicle_global_pos.alt) ? _vehicle_global_pos.alt : 0.0f;
		const float rel_alt_m = std::isfinite(_vehicle_local_pos.z) ? -_vehicle_local_pos.z : 0.0f;
		const float vx_m = std::isfinite(_vehicle_local_pos.vx) ? _vehicle_local_pos.vx : 0.0f;
		const float vy_m = std::isfinite(_vehicle_local_pos.vy) ? _vehicle_local_pos.vy : 0.0f;
		const float vz_m = std::isfinite(_vehicle_local_pos.vz) ? _vehicle_local_pos.vz : 0.0f;

		const int32_t lat = clamp_i32((int64_t)llround(lat_deg * 1e7));
		const int32_t lon = clamp_i32((int64_t)llround(lon_deg * 1e7));
		const int32_t alt_mm = clamp_i32((int64_t)llround(alt_m * 1000.0f));
		const int32_t rel_alt_mm = clamp_i32((int64_t)llround(rel_alt_m * 1000.0f));
		const int16_t vx = clamp_i16((int32_t)lroundf(vx_m * 100.0f));
		const int16_t vy = clamp_i16((int32_t)lroundf(vy_m * 100.0f));
		const int16_t vz = clamp_i16((int32_t)lroundf(vz_m * 100.0f));

		uint8_t payload[32] {};
		size_t offset = 0;
		append_i32(payload, offset, lat);
		append_i32(payload, offset, lon);
		append_i32(payload, offset, alt_mm);
		append_i32(payload, offset, rel_alt_mm);
		append_i16(payload, offset, vx);
		append_i16(payload, offset, vy);
		append_i16(payload, offset, vz);
		send_msg(kMsgGlobalPos, payload, (uint8_t)offset);
		_last_global_us = now;
	}
//smp_global_position() # 1366
	if (_have_vehicle_attitude && _have_vehicle_ang_vel && (now - _last_att_us) >= 20000) {
		const matrix::Quatf q(_vehicle_attitude.q);
		const matrix::Eulerf e(q);

		const float roll_mrad = e.phi() * 1000.0f;
		const float pitch_mrad = e.theta() * 1000.0f;
		const float yaw_mrad = matrix::wrap_2pi(e.psi()) * 1000.0f;

		const int16_t roll_i = clamp_i16((int32_t)lroundf(roll_mrad));
		const int16_t pitch_i = clamp_i16((int32_t)lroundf(pitch_mrad));
		const uint16_t yaw_u = clamp_u16((int32_t)lroundf(yaw_mrad));
		const int16_t rs = clamp_i16((int32_t)lroundf(_vehicle_ang_vel.xyz[0] * 1000.0f));
		const int16_t ps = clamp_i16((int32_t)lroundf(_vehicle_ang_vel.xyz[1] * 1000.0f));
		const int16_t ys = clamp_i16((int32_t)lroundf(_vehicle_ang_vel.xyz[2] * 1000.0f));

		uint8_t payload[16] {};
		size_t offset = 0;
		append_i16(payload, offset, roll_i);
		append_i16(payload, offset, pitch_i);
		append_u16(payload, offset, yaw_u);
		append_i16(payload, offset, rs);
		append_i16(payload, offset, ps);
		append_i16(payload, offset, ys);
		send_msg(kMsgAttitude, payload, (uint8_t)offset);
		_last_att_us = now;
	}
// smp_attitude() #1389~1393
	if (_have_vehicle_local_pos && (now - _last_heading_us) >= 200000) {
		const float rad2deg = 57.2957795f;
		const float heading_rad = std::isfinite(_vehicle_local_pos.heading) ? _vehicle_local_pos.heading : 0.0f;
		float heading_deg = heading_rad * rad2deg;
		if (heading_deg < 0.0f) {
			heading_deg += 360.0f;
		}
		const uint16_t heading_cd = clamp_u16((int32_t)lroundf(heading_deg * 100.0f));

		uint8_t payload[8] {};
		size_t offset = 0;
		append_u16(payload, offset, heading_cd);
		append_u16(payload, offset, heading_cd);
		send_msg(kMsgHeading, payload, (uint8_t)offset);
		_last_heading_us = now;
	}
//smp_heading() #1408~1410
	if (_have_vehicle_local_pos && (now - _last_air_us) >= 200000) {
		const float vx_m = std::isfinite(_vehicle_local_pos.vx) ? _vehicle_local_pos.vx : 0.0f;
		const float vy_m = std::isfinite(_vehicle_local_pos.vy) ? _vehicle_local_pos.vy : 0.0f;
		const float vz_m = std::isfinite(_vehicle_local_pos.vz) ? _vehicle_local_pos.vz : 0.0f;
		const float groundspeed = std::sqrt((vx_m * vx_m) + (vy_m * vy_m));
		const uint16_t groundspeed_cs = clamp_u16((int32_t)lroundf(groundspeed * 100.0f));
		const int16_t climb_cs = clamp_i16((int32_t)lroundf((-vz_m) * 100.0f));
		float airspeed_m_s = 0.0f;
		if (_have_airspeed && std::isfinite(_airspeed.indicated_airspeed_m_s)) {
			airspeed_m_s = _airspeed.indicated_airspeed_m_s;
		}
		const uint16_t airspeed_cs = clamp_u16((int32_t)lroundf(airspeed_m_s * 100.0f));

		float throttle_norm = 0.0f;
		if (_have_thrust_sp) {
			const float tx = std::isfinite(_thrust_sp.xyz[0]) ? _thrust_sp.xyz[0] : 0.0f;
			const float ty = std::isfinite(_thrust_sp.xyz[1]) ? _thrust_sp.xyz[1] : 0.0f;
			const float tz = std::isfinite(_thrust_sp.xyz[2]) ? _thrust_sp.xyz[2] : 0.0f;
			const float mag = std::sqrt((tx * tx) + (ty * ty) + (tz * tz));
			if (std::isfinite(mag)) {
				throttle_norm = mag;
			}
		}
		if (throttle_norm < 0.0f) {
			throttle_norm = 0.0f;
		} else if (throttle_norm > 1.0f) {
			throttle_norm = 1.0f;
		}
		const uint8_t throttle_pct = clamp_u8((int32_t)lroundf(throttle_norm * 100.0f));

		const int32_t alt_mm = _have_vehicle_global_pos
					       ? clamp_i32((int64_t)llround(std::isfinite(_vehicle_global_pos.alt) ? _vehicle_global_pos.alt * 1000.0f : 0.0f))
					       : clamp_i32((int64_t)llround(std::isfinite(_vehicle_local_pos.z) ? -_vehicle_local_pos.z * 1000.0f : 0.0f));

		uint8_t payload[16] {};
		size_t offset = 0;
		append_u16(payload, offset, airspeed_cs);
		append_u16(payload, offset, groundspeed_cs);
		append_i16(payload, offset, climb_cs);
		append_u8(payload, offset, throttle_pct);
		append_i32(payload, offset, alt_mm);
		send_msg(kMsgAirdata, payload, (uint8_t)offset);
		_last_air_us = now;
	}
}

// send_msg():
// SMP 프레임(sync+header+payload+crc16)을 구성해 송신합니다.
// 텔레메트리/ACK/테스트 송신 등 공통 TX 함수입니다.
void Smp::send_msg(uint8_t msg_id, const uint8_t *payload, uint8_t payload_len)
{
	if (_fd < 0 || payload_len > kMaxPayload) {
		return;
	}

	uint8_t frame[kMaxFrame] {};
	frame[0] = kSync1;
	frame[1] = _sysid;
	frame[2] = _compid;
	frame[3] = _tx_seq++;
	frame[4] = msg_id;
	frame[5] = payload_len;
	memcpy(&frame[6], payload, payload_len);

	uint8_t crc_buf[5 + kMaxPayload] {};
	crc_buf[0] = frame[1];
	crc_buf[1] = frame[2];
	crc_buf[2] = frame[3];
	crc_buf[3] = frame[4];
	crc_buf[4] = frame[5];
	memcpy(&crc_buf[5], payload, payload_len);

	uint16_t crc = crc16_ccitt(crc_buf, 5 + payload_len);
	frame[6 + payload_len] = (uint8_t)(crc & 0xFF);
	frame[6 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

	ssize_t nwritten = send_bytes(frame, 1 + 1 + 1 + 1 + 1 + 1 + payload_len + 2);

	if (nwritten < 0) {
		if ((_link_mode == LinkMode::TcpServer || _link_mode == LinkMode::TcpClient) && !is_would_block(errno)) {
			PX4_WARN("tcp tx failed: %d", errno);
			close_client();
		} else if (_link_mode == LinkMode::UdpServer || _link_mode == LinkMode::UdpClient) {
			if (!is_would_block(errno)) {
				PX4_WARN("udp tx failed: %d", errno);
			}
		} else if (_link_mode == LinkMode::Serial) {
			PX4_ERR("write failed: %d", errno);
		}
	}
}

ssize_t Smp::send_bytes(const uint8_t *buf, size_t len)
{
	if (_fd < 0) {
		return -1;
	}

#if defined(__PX4_POSIX)
	if (_link_mode == LinkMode::UdpServer) {
		if (!_udp_peer_valid) {
			return 0;
		}
		return ::sendto(_fd, buf, len, 0, (sockaddr *)&_udp_peer, sizeof(_udp_peer));
	}
#endif

	return ::write(_fd, buf, len);
}

// Append little-endian u16 to payload buffer.
// payload 버퍼에 리틀엔디안 u16을 추가합니다. @@@@
void Smp::append_u16(uint8_t *buf, size_t &offset, uint16_t value)
{
	buf[offset++] = (uint8_t)(value & 0xFF);
	buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
}

// Append little-endian i16 to payload buffer.
// payload 버퍼에 리틀엔디안 i16을 추가합니다.
void Smp::append_i16(uint8_t *buf, size_t &offset, int16_t value)
{
	const uint16_t u = (uint16_t)value;
	buf[offset++] = (uint8_t)(u & 0xFF);
	buf[offset++] = (uint8_t)((u >> 8) & 0xFF);
}

// Append little-endian u32 to payload buffer.
// payload 버퍼에 리틀엔디안 u32를 추가합니다.
void Smp::append_u32(uint8_t *buf, size_t &offset, uint32_t value)
{
	buf[offset++] = (uint8_t)(value & 0xFF);
	buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 16) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 24) & 0xFF);
}

// Append little-endian i32 to payload buffer.
// payload 버퍼에 리틀엔디안 i32를 추가합니다.
void Smp::append_i32(uint8_t *buf, size_t &offset, int32_t value)
{
	const uint32_t u = (uint32_t)value;
	buf[offset++] = (uint8_t)(u & 0xFF);
	buf[offset++] = (uint8_t)((u >> 8) & 0xFF);
	buf[offset++] = (uint8_t)((u >> 16) & 0xFF);
	buf[offset++] = (uint8_t)((u >> 24) & 0xFF);
}

// Append little-endian u64 to payload buffer.
// payload 버퍼에 리틀엔디안 u64를 추가합니다.
void Smp::append_u64(uint8_t *buf, size_t &offset, uint64_t value)
{
	buf[offset++] = (uint8_t)(value & 0xFF);
	buf[offset++] = (uint8_t)((value >> 8) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 16) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 24) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 32) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 40) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 48) & 0xFF);
	buf[offset++] = (uint8_t)((value >> 56) & 0xFF);
}

// Append u8 to payload buffer.
// payload 버퍼에 u8을 추가합니다.
void Smp::append_u8(uint8_t *buf, size_t &offset, uint8_t value)
{
	buf[offset++] = value;
}

// Append little-endian f32 to payload buffer.
// payload 버퍼에 리틀엔디안 f32를 추가합니다.
void Smp::append_f32(uint8_t *buf, size_t &offset, float value)
{
	memcpy(&buf[offset], &value, sizeof(value));
	offset += sizeof(value);
}

// Read little-endian f32 from payload buffer.
// payload 버퍼에서 리틀엔디안 f32를 읽습니다.
float Smp::read_f32(const uint8_t *buf)
{
	float value = 0.0f;
	memcpy(&value, buf, sizeof(value));
	return value;
}

// Read little-endian i32 from payload buffer.
// payload 버퍼에서 리틀엔디언 i32를 읽습니다.
int32_t Smp::read_i32(const uint8_t *buf)
{
	int32_t value = 0;
	memcpy(&value, buf, sizeof(value));
	return value;
}

// Read little-endian f64 from payload buffer.
// payload 버퍼에서 리틀엔디안 f64를 읽습니다.
double Smp::read_f64(const uint8_t *buf)
{
	double value = 0.0;
	memcpy(&value, buf, sizeof(value));
	return value;
}

// Compute CRC-16/CCITT-FALSE over the provided bytes.
// 주어진 바이트에 대해 CRC-16/CCITT-FALSE를 계산합니다.
uint16_t Smp::crc16_ccitt(const uint8_t *data, size_t len) const
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x8000) {
				crc = (uint16_t)((crc << 1) ^ 0x1021);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

bool Smp::parse_device(const char *device, LinkMode &mode, char *host, size_t host_len, uint16_t &port)
{
	mode = LinkMode::Serial;
	port = 0;
	if (!device || !host || host_len == 0) {
		return false;
	}

	const char *addr = nullptr;
	bool is_tcp = false;

	if (strncmp(device, "tcp://", 6) == 0) {
		is_tcp = true;
		addr = device + 6;
	} else if (strncmp(device, "udp://", 6) == 0) {
		addr = device + 6;
	} else {
		return true;
	}

	const char *colon = strrchr(addr, ':');
	if (!colon) {
		return false;
	}

	const char *port_str = colon + 1;
	if (!port_str || *port_str == '\0') {
		return false;
	}

	unsigned long p = strtoul(port_str, nullptr, 10);
	if (p == 0 || p > 65535) {
		return false;
	}
	port = (uint16_t)p;

	size_t len = (size_t)(colon - addr);
	if (len >= host_len) {
		return false;
	}

	if (len == 0) {
		mode = is_tcp ? LinkMode::TcpServer : LinkMode::UdpServer;
		strncpy(host, "0.0.0.0", host_len - 1);
		host[host_len - 1] = '\0';
		return true;
	}

	memcpy(host, addr, len);
	host[len] = '\0';

	if (strcmp(host, "*") == 0 || strcmp(host, "0.0.0.0") == 0) {
		mode = is_tcp ? LinkMode::TcpServer : LinkMode::UdpServer;
	} else {
		mode = is_tcp ? LinkMode::TcpClient : LinkMode::UdpClient;
	}

	return true;
}

bool Smp::set_nonblocking(int fd)
{
#if !defined(__PX4_POSIX)
	(void)fd;
	return true;
#else
	int flags = fcntl(fd, F_GETFL, 0);
	if (flags < 0) {
		return false;
	}
	if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
		return false;
	}
	return true;
#endif
}
