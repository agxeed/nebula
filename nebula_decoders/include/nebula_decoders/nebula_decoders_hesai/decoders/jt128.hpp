// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_sensor.hpp"

namespace nebula::drivers
{

namespace hesai_packet
{

#pragma pack(push, 1)

struct TailJT128
{
  uint8_t reserved1[11];
  uint8_t working_mode;
  uint8_t return_mode;
  uint16_t motor_speed;
  DateTime<1900> date_time;
  // SecondsSinceEpoch date_time;
  uint32_t timestamp;
  uint8_t factory_information;
  // uint8_t high_temperature_shutdown_flag;
  uint8_t reserved2[30];
  // uint16_t motor_speed;
  
  
  // uint8_t factory_information;
  // SecondsSinceEpoch date_time;

  /* Ignored optional fields */

  // uint32_t udp_sequence;
  // uint32_t crc_tail;
};

struct PacketJT128 : public PacketBase<2, 128, 2, 100>
{
  using body_t =
    Body<Block<Unit4B, PacketJT128::n_channels>, PacketJT128::n_blocks>;
  Header12B header;
  body_t body;
  uint32_t crc_body;
  TailJT128 tail;

  /* Ignored optional fields */

  // uint8_t cyber_security[32];
};

#pragma pack(pop)

}  // namespace hesai_packet

class JT128
: public HesaiSensor<hesai_packet::PacketJT128>
{
private:
  static constexpr int firing_time_offset_ns[128] = {
    95180, 23240, 98220, 20200, 101260, 17160, 104300, 14120,
    77280, 92140, 74240, 89100, 71200, 86060, 68160, 83020,
    50260, 11080, 47220, 8040, 44180, 5000, 41140, 1960,
    65120, 105820, 62080, 102780, 59040, 99740, 56000, 96700,
    38100, 24760, 35060, 21720, 32020, 18680, 28980, 15640,
    78800, 93660, 75760, 90620, 72720, 87580, 69680, 84540,
    51780, 12600, 48740, 9560, 45700, 6520, 42660, 3480,
    66640, 103540, 63600, 100500, 60560, 97460, 57520, 94420,
    39620, 22480, 36580, 19440, 33540, 16400, 30500, 13360,
    76520, 91380, 73480, 88340, 70440, 85300, 67400, 82260,
    49500, 10320, 46460, 7280, 43420, 4240, 40380, 1200,
    64360, 105060, 61320, 102020, 58280, 98980, 55240, 95940,
    37340, 24000, 34300, 20960, 31260, 17920, 28220, 14880,
    78040, 92900, 75000, 89860, 71960, 86820, 68920, 83780,
    51020, 11840, 47980, 8800, 44940, 5760, 41900, 2720,
    65880, 62840, 59800, 56760, 38860, 35820, 32780, 29740,
  };

public:
  static constexpr float min_range = 1.f;
  static constexpr float max_range = 60.0f;
  static constexpr size_t max_scan_buffer_points = 230400;
  static constexpr FieldOfView<int32_t, MilliDegrees> fov_mdeg{
    {0, 360'000},       // azimuth: 0°–360°
    {-4'400, 90'500}    // elevation: -4.4° – 90.5°
  };
  static constexpr AnglePair<int32_t, MilliDegrees> peak_resolution_mdeg{400, 740};

  int get_packet_relative_point_time_offset(
    uint32_t block_id, uint32_t channel_id, const packet_t & packet) override
  {
    const auto n_returns = hesai_packet::get_n_returns(packet.tail.return_mode);

    int block_offset_ns = 0;

    if (n_returns == 1) {
      // Single return: Block 0 and Block 1 have different t(m)
      if (block_id == 0) {
        block_offset_ns = -1'999'111;  // ≈ -1999.111 μs
      } else {
        block_offset_ns = -1'888'000;  // -1888 μs
      }
    } else {
      // Dual return: both blocks share the same t(m)
      block_offset_ns = -1'888'000;    // -1888 μs
    }

    return block_offset_ns + firing_time_offset_ns[channel_id];
  }
};

}  // namespace nebula::drivers
