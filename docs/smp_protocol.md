# SMP Protocol (v1)

This document defines the custom SMP binary protocol used between a PC
and PX4. The uORB messages remain unchanged; the PX4 module bridges uORB to
SMP frames. There is no version field in the frame.

## 요약 (한국어)

SMP는 PC와 PX4 사이에서 사용하는 사용자 정의 바이너리 프로토콜입니다.
uORB 메시지는 그대로 유지하고, PX4의 SMP 모듈이 uORB와 SMP 프레임 사이를
브리지합니다. 프레임은 sync(0xA5), sysid, comid, seq, msgid, len, payload,
crc16 순서이며 **버전 필드는 없습니다**. 모든 멀티바이트 필드는
little-endian입니다. CRC는 CRC-16/CCITT-FALSE(초기값 0xFFFF)로
`sysid | comid | seq | msgid | len | payload` 구간에 대해 계산합니다.

## Data Flow Overview

```
PC/Companion
  |  SMP frame (UART/USB)
  v
PX4 SMP module (src/modules/smp)
  |  publish/subscribe
  v
uORB topics (smp_in/smp_out, vehicle_command, vehicle_status, ...)
```

```
PX4 uORB telemetry
  |  smp_out
  v
PX4 SMP module
  |  SMP frame (UART/USB)
  v
PC/Companion
```

## Frame Fields (Table)

| Field | Size (bytes) | Type | Description |
| --- | --- | --- | --- |
| sync | 1 | uint8 | Fixed 0xA5 |
| sysid | 1 | uint8 | Sender system id |
| comid | 1 | uint8 | Sender component id |
| seq | 1 | uint8 | Sequence number (increment per sender) |
| msgid | 1 | uint8 | Message id |
| len | 1 | uint8 | Payload length |
| payload | N | bytes | Variable payload |
| crc16 | 2 | uint16 | CRC-16/CCITT-FALSE over `sysid..payload` |

## Frame Format

All multi-byte fields are little-endian.

```
sync(0xA5) | sysid | comid | seq | msgid(uint8) | len(uint8) | payload | crc16(uint16)
```

## Encoder/Decoder Rules

### Encoder (TX)
- Build header: `sync | sysid | comid | seq | msgid | len`
- Append `payload` (len bytes)
- Compute CRC16-CCITT-FALSE (init 0xFFFF) over `sysid..payload` (exclude sync)
- Append `crc16` (little-endian)

### Decoder (RX)
- Scan stream for `sync(0xA5)`
- Ensure minimum frame length, then read `len`
- Verify full frame is available: `1+1+1+1+1+1+len+2`
- Recompute CRC over `sysid..payload` and compare to received CRC
- On CRC failure, drop 1 byte and rescan

### Data Rules
- Endianness: little-endian for all multi-byte fields
- `seq`: uint8, wraps 0..255 (sender increments)
- `len`: payload length only (excludes header and CRC)
- Max payload (current spec): 31 bytes

## Message Summary (Table)

| MsgID | Name | Direction | Payload size (bytes) | uORB source/sink |
| --- | --- | --- | --- | --- |
| 0x01 | HEARTBEAT | PX4 -> PC | 5 | `vehicle_status` |
| 0x02 | VEH_DATA | PX4 -> PC | 11 | params (`SMP_MAX_SPD`, `SMP_MAX_ALT`, `SMP_ENDUR`, `SMP_PAYLOAD`, `SMP_FW_VER`) |
| 0x10 | SYS_STATUS | PX4 -> PC | 9 | `battery_status`, `cpuload`, `health_report` |
| 0x11 | BATTERY_STATUS | PX4 -> PC | 9 | `battery_status` |
| 0x20 | GPS_FIX | PX4 -> PC | 10 | `vehicle_gps_position` |
| 0x21 | GLOBAL_POSITION | PX4 -> PC | 22 | `vehicle_global_position`, `vehicle_local_position` |
| 0x30 | ATTITUDE | PX4 -> PC | 12 | `vehicle_attitude`, `vehicle_angular_velocity` |
| 0x31 | HEADING | PX4 -> PC | 4 | `vehicle_local_position` |
| 0x40 | AIRDATA | PX4 -> PC | 11 | `airspeed`, `vehicle_local_position`, `vehicle_global_position`, `vehicle_thrust_setpoint` |
| 0x90 | COMMAND_ACK | PX4 -> PC | 3 | `vehicle_command_ack` |
| 0x91 | COMMAND_DO | PC -> PX4 | 7 | `vehicle_command` |
| 0x92 | COMMAND_SHORT | PC -> PX4 | 19 | `vehicle_command` |
| 0x93 | COMMAND_LONG | PC -> PX4 | 31 | `vehicle_command` |
| 0xB0 | PARAM_SET | PC -> PX4 | 23 | (TBD, not implemented) |
| 0xB1 | PARAM_GET | PC -> PX4 | 18 | (TBD, not implemented) |

## SMP ↔ MAVLink Command Mapping

SMP commands are converted to PX4 `vehicle_command` (MAVLink-equivalent) and
`vehicle_command_ack` is converted back to SMP COMMAND_ACK (0x90).

### SMP -> MAVLink (vehicle_command)

| SMP cmd_id | Meaning | MAVLink command |
| --- | --- | --- |
| 1 | ARM/DISARM | `MAV_CMD_COMPONENT_ARM_DISARM` |
| 2 | SET_MODE | `MAV_CMD_DO_SET_MODE` |
| 3 | TAKEOFF | `MAV_CMD_NAV_TAKEOFF` |
| 50 | GOTO | `MAV_CMD_DO_REPOSITION` |

Parameter mapping:
- COMMAND_DO (msgid 0x91): `cmd_id` + `p1(int32)` -> `param1`, `target_sys/comp` -> target.
- COMMAND_SHORT (msgid 0x92): `p1..p4(int32)` -> `param1..param4`.
- COMMAND_LONG (msgid 0x93): `p1..p7(int32)` -> `param1..param7`.
- ARM/DISARM: `p1` encodes action/force: `0=disarm`, `1=arm`, `2=force disarm`, `3=force arm`
  (`bit0=action`, `bit1=force`). This maps to `param1` (0/1) and `param2=21196` when force.
- SET_MODE: `p1` (0..8) mapped to PX4 custom main/sub modes, then
  `param1=1` (custom enabled), `param2=main_mode`, `param3=sub_mode`.
- TAKEOFF: `p1=rel_alt_m` (meters, int32). If global altitude is available,
  `param7 = current_alt_m + p1` (absolute AMSL); otherwise `param7 = p1`.
- GOTO: uses `p1=lat_e7`, `p2=lon_e7`, `p3=alt_mm` and **`p4 unused`**.
  These are converted to `param5=lat_deg`, `param6=lon_deg`, `param7=alt_m`.
  `param4` is set to `NAN` (yaw unused).
  `param2` is fixed to `1.0` (change_mode_flags bit0=1) so PX4 accepts
  `MAV_CMD_DO_REPOSITION` and switches to AUTO_LOITER.

### MAVLink -> SMP (COMMAND_ACK)

PX4 `vehicle_command_ack` is converted to SMP COMMAND_ACK (msgid 0x90):

Payload: `cmd_id | result | seq`
- `cmd_id` is mapped from MAVLink command using the table above.
- If the MAVLink command is not in the table, `cmd_id = (mav_cmd & 0xFF)`.
- `result` is the MAVLink `MAV_RESULT` value (0..5).
- `seq` is the SMP sequence number remembered for that `cmd_id` (if available).

Note: The optional SMP "echo" (`smp start -e -a 0x90 -p ACK`) is a debug-only
response and is not a real COMMAND_ACK. It uses msgid 0x90 with payload "ACK".

## Message IDs and Payloads

### 0x01 HEARTBEAT
Payload:
- type: uint8
- autopilot: uint8
- base_mode: uint8
- custom_mode: uint8
- system_status: uint8

Notes:
- autopilot: currently set to 1 (PX4). Change if you need a different enum.
- base_mode: 0 = disarmed, 1 = armed (current implementation)

Type enum:
- 0: GENERIC
- 1: FIXED_WING
- 2: QUADROTOR

Custom_mode enum:
- 0: STABILIZE
- 1: ACRO
- 2: ALT_HOLD
- 3: AUTO
- 4: GUIDED
- 5: LOITER
- 6: RTL (Return to Launch)
- 7: CIRCLE
- 8: LAND

PX4 nav_state mapping (implementation note):
- MANUAL/STAB -> STABILIZE (0)
- ACRO -> ACRO (1)
- ALTCTL/ALTITUDE_CRUISE -> ALT_HOLD (2)
- AUTO_* -> AUTO (3)
- OFFBOARD -> GUIDED (4)
- POSCTL/POSITION_SLOW/AUTO_LOITER -> LOITER (5)
- AUTO_RTL -> RTL (6)
- ORBIT -> CIRCLE (7)
- AUTO_LAND/DESCEND -> LAND (8)

System_status enum:
- 0: UNINIT
- 1: BOOT (System is booting)
- 2: CALIBRATING
- 3: STANDBY (Ready, disarmed)
- 4: ACTIVE (Armed and active)
- 5: CRITICAL (Failure, unsafe)
- 6: EMERGENCY
- 7: POWEROFF (Shutting down)
- 8: FLIGHT_TERMINATION

### 0x02 VEH_DATA
Payload:
- vehicle_type: uint8
- max_speed: uint16
- max_altitude: uint16
- endurance: uint16
- payload_capacity: uint16
- fw_version: uint16

Notes:
- max_speed: m/s
- max_altitude: meters
- endurance: minutes
- payload_capacity: grams
- fw_version: uint16 (encoding TBD)
- Values are read from parameters:
  - `SMP_MAX_SPD` (max_speed)
  - `SMP_MAX_ALT` (max_altitude)
  - `SMP_ENDUR` (endurance)
  - `SMP_PAYLOAD` (payload_capacity)
  - `SMP_FW_VER` (fw_version)

### 0x10 SYS_STATUS
Payload:
- sens_health: uint8
- cpu_load: uint8
- voltage_batt: uint16
- current_batt: int16
- battery_rem: uint8
- errors_comm: uint16

sens_health bits:
- bit0 (0x01): GPS healthy
- bit1 (0x02): IMU/Gyro healthy
- bit2 (0x04): Accelerometer healthy
- bit3 (0x08): Magnetometer/Compass healthy
- bit4 (0x10): Barometer healthy
- bit5 (0x20): Optical Flow healthy
- bit6 (0x40): Vision/Camera healthy
- bit7 (0x80): Rangefinder/Lidar healthy

Notes:
- cpu_load: 0..100
- voltage_batt: mV
- current_batt: mA
- battery_rem: 0..100 (%)
- errors_comm: 0..65535
- cpu_load is derived from `cpuload.load` (uORB `cpuload`)
- sens_health is derived from uORB `health_report` flags

### 0x11 BATTERY_STATUS
Payload:
- voltage: uint16
- current: int16
- battery_rem: uint8
- time_rem: uint16
- cell_count: uint8
- temp: int8

### 0x20 GPS_FIX
Payload:
- fix_type: uint8
- satellites_visible: uint8
- hdop: uint16
- vdop: uint16
- eph: uint16
- epv: uint16

fix_type enum:
- 0: NO_FIX
- 1: 2D_FIX
- 2: 3D_FIX
- 3: DGPS
- 4: RTK_FLOAT
- 5: RTK_FIXED

PX4 fix_type mapping (implementation note):
- PX4 0/1 (NONE) -> 0 (NO_FIX)
- PX4 2 (2D) -> 1 (2D_FIX)
- PX4 3 (3D) -> 2 (3D_FIX)
- PX4 4 (RTCM/DGPS) -> 3 (DGPS)
- PX4 5 (RTK_FLOAT) -> 4 (RTK_FLOAT)
- PX4 6 (RTK_FIXED) -> 5 (RTK_FIXED)
- PX4 8 (EXTRAPOLATED) -> 2 (3D_FIX)

### 0x21 GLOBAL_POSITION
Payload:
- lat: int32
- lon: int32
- alt: int32
- relative_alt: int32
- vx: int16
- vy: int16
- vz: int16

### 0x30 ATTITUDE
Payload:
- roll: int16
- pitch: int16
- yaw: uint16
- rollspeed: int16
- pitchspeed: int16
- yawspeed: int16

Notes:
- voltage: mV
- current: mA
- battery_rem: 0..100 (%)
- time_rem: seconds
- temp: °C
- roll/pitch: mrad (-32768..32767)
- yaw: mrad (0..6283)
- rates: mrad/s

### 0x31 HEADING
Payload:
- heading: uint16
- heading_target: uint16

Notes:
- heading range: 0..36000 (0..360 deg, centi-deg)

### 0x40 AIRDATA
Payload:
- airspeed: uint16
- groundspeed: uint16
- climb_rate: int16
- throttle: uint8
- altitude_amsl: int32

Notes:
- airspeed/groundspeed: cm/s
- climb_rate: cm/s
- throttle: 0..100
- altitude_amsl: mm
- airspeed: from uORB `airspeed.indicated_airspeed_m_s`
- throttle: from uORB `vehicle_thrust_setpoint` magnitude (0..1 -> 0..100)

### 0x90 COMMAND_ACK
Payload:
- command: uint8
- result: uint8
- seq: uint8

result enum:
- 0: ACCEPTED
- 1: TEMPORARILY_REJECTED
- 2: DENIED
- 3: UNSUPPORTED
- 4: FAILED
- 5: IN_PROGRESS

### 0x91 COMMAND_DO
Payload:
- cmd_id: uint8
- p1: int32
- target_system: uint8
- target_component: uint8

### 0x92 COMMAND_SHORT
Payload:
- cmd_id: uint8
- p1: int32
- p2: int32
- p3: int32
- p4: int32
- target_system: uint8
- target_component: uint8

### 0x93 COMMAND_LONG
Payload:
- cmd_id: uint8
- p1: int32
- p2: int32
- p3: int32
- p4: int32
- p5: int32
- p6: int32
- p7: int32
- target_system: uint8
- target_component: uint8

### 0xB0 PARAM_SET
Payload:
- target_system: uint8
- target_component: uint8
- param_id: char[16]
- param_value: float32
- param_type: uint8

### 0xB1 PARAM_GET
Payload:
- target_system: uint8
- target_component: uint8
- param_id: char[16]

## Scaling / Units (Implementation Note)

The current PX4 SMP implementation uses the following conversions. Adjust if you
want different units.
- GLOBAL_POSITION: lat/lon = degrees * 1e7, alt/relative_alt = millimeters, vx/vy/vz = cm/s
- ATTITUDE: angles = milliradians (rad * 1000), rates = mrad/s
- HEADING: centi-degrees (deg * 100)
- GPS_FIX: hdop/vdop/eph/epv = value * 100 (0.01 resolution)
- AIRDATA: speeds = cm/s, climb_rate = cm/s, altitude_amsl = millimeters
- SYS_STATUS/BATTERY_STATUS: voltage = mV, current = mA, battery_rem = percent, time_rem = seconds, temp = °C
