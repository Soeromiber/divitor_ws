#pragma once

#include <array>
#include <cstdint>
#include <iostream>

#include "MLX90640_API.h"

#define MLX_I2C_ADDR 0x33

class MLX90640Camera {
  public:
    const int WIDTH = 32;
    const int HEIGHT = 24;

    bool Open(uint8_t i2c_address, uint8_t refresh_rate,
              float emissivity = 0.95f, float reflected_temperature = 23.0f);
    bool ReadFrame(float *temperatures);
    bool opened() const;

  private:
    bool opened_ = false;
    uint8_t i2c_address_;
    float emissivity_;
    float reflected_temperature_;
    std::array<uint16_t, 832> eeprom_{};
    std::array<uint16_t, 834> frame_{};
    paramsMLX90640 params_{};
};
