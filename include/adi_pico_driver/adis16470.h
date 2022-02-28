// Copyright (c) 2017, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef ADI_DRIVER_ADIS16470_H
#define ADI_DRIVER_ADIS16470_H

#include <termios.h>
#include <array>
#include <string>

#include "adi_pico_driver/serial_port.h"

class Adis16470
{
public:
  // Gyro sensor(x, y, z)
  std::array<double, 3> gyro;
  // Acceleration sensor(x, y, z)
  std::array<double, 3> accl;
  // Temperature sensor
  double temp;

  Adis16470();
  ~Adis16470();
  int open_port(const std::string &device);

  int get_product_id(int16_t &);
  int update(void);
  int update_burst(void);
  int bias_correction_update(void);
  int set_bias_estimation_time(int16_t tbc);
  int set_filt_ctrl(const int16_t filt);
  int set_dec_rate(const int16_t rate);

private:
  bool write_register(const uint8_t address, const int16_t);
  bool read_register(const uint8_t address, int16_t &);
  bool read_register_half_transaction(const uint8_t address, int16_t &);
  bool init_usb_iss();
  bool initAdis16470();
  int16_t big_endian_to_short(const uint8_t *buf);

  SerialPort serial_port;
  double accl_scale_factor;
  double k_g;
};

#endif // ADI_DRIVER_ADIS16470_H
