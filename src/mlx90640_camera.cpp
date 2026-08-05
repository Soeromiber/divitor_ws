#include "divitor_driver/mlx90640_camera.hpp"

#include "MLX90640_I2C_Driver.h"

bool MLX90640Camera::Open(uint8_t i2c_address, uint8_t refresh_rate,
                          float emissivity, float reflected_temperature) {
    i2c_address_ = i2c_address;
    emissivity_ = emissivity;
    reflected_temperature_ = reflected_temperature;

    MLX90640_I2CInit();

    std::cout << "Reading sensor EEPROM calibration data..." << std::endl;
    if (MLX90640_DumpEE(i2c_address_, eeprom_.data()) != 0) {
        return false;
    }

    MLX90640_ExtractParameters(eeprom_.data(), &params_);

    if (MLX90640_SetRefreshRate(i2c_address_, refresh_rate) != 0) {
        return false;
    }

    opened_ = true;
    return true;
}

bool MLX90640Camera::ReadFrame(float *temperatures) {
    if (MLX90640_GetFrameData(i2c_address_, frame_.data()) < 0) {
        return false;
    }

    MLX90640_CalculateTo(frame_.data(), &params_, emissivity_,
                         reflected_temperature_, temperatures);
    return true;
}

bool MLX90640Camera::opened() const { return opened_; }
