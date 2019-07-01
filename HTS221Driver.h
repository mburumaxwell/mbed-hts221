#ifndef __HTS221_DRIVER_H
#define __HTS221_DRIVER_H
#include <mbed.h>

class HTS221Driver {
public:
    HTS221Driver(I2C *driver, int32_t addr = 0xBE)
    {
        this->i2c_driver = driver;
        this->dev_addr = addr;
    }
    int32_t init();
    int32_t set_data_ready(bool enable = false, bool openDrain = false, bool activeLow = false);
    int32_t read_id(uint8_t *id);
    int32_t read_humidity(float *humidity);
    int32_t read_temperature(float *temperature);

private:
    I2C *i2c_driver;
    int32_t dev_addr;
};

#endif /* __HTS221_DRIVER_H */