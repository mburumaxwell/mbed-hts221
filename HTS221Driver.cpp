#include "hts221_def.h"
#include "HTS221Driver.h"
#include "mbed_debug.h"

#define DEBUG_ERR_WRITE(a1, a2)  debug_if(a1, "HTS221 write 0x%02X failed with result %d", a2, a1)
#define DEBUG_ERR_READ(a1, a2)   debug_if(a1, "HTS221 read 0x%02X failed with result %d", a2, a1)

int32_t HTS221Driver::init()
{
    int ret = 0;
    char reg, val;
  
    /* Read CTRL_REG1 */
    reg = HTS221_REGISTER_CTRL_REG1;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;
  
    /* Enable BDU */
    val &= ~HTS221_BDU_MASK;
    val |= (1 << HTS221_BDU_BIT);
  
    /* Set default ODR */
    val &= ~HTS221_ODR_MASK;
    val |= (uint8_t)0x01; /* Set ODR to 1Hz */

    /* Activate the device */
    val |= HTS221_PD_MASK;
  
    /* Apply settings to CTRL_REG1 */
    reg = HTS221_REGISTER_CTRL_REG1;
    char buf[2] = { reg, val };
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, reg);
    return ret;
}

int32_t HTS221Driver::set_data_ready(bool enable, bool openDrain, bool activeLow)
{
    int ret = 0;
    char reg, val;
  
    /* Read CTRL_REG3 */
    reg = HTS221_REGISTER_CTRL_REG3;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;
  
    /* Set DRDY_EN */
    val &= ~HTS221_DRDY_MASK;
    val |= (enable << HTS221_DRDY_BIT); // 0 = disable, 1 = enable
  
    /* Set PP_OD */
    val &= ~HTS221_PP_OD_MASK;
    val |= (openDrain << HTS221_PP_OD_BIT); // 0 = push-pull, 1 = open drain

    /* Set DRDY_H_L */
    val &= ~HTS221_DRDY_H_L_MASK;
    val |= (activeLow << HTS221_DRDY_H_L_BIT); // 0 = active high, 1 = active low
  
    /* Apply settings to CTRL_REG3 */
    reg = HTS221_REGISTER_CTRL_REG3;
    char buf[2] = { reg, val };
    ret = i2c_driver->write(dev_addr, buf, 2);
    DEBUG_ERR_WRITE(ret, reg);
    return ret;
}

int32_t HTS221Driver::read_id(uint8_t *id)
{  
    int ret = 0;
    char reg, val;
  
    /* Read WHO_AM_I */
    reg = HTS221_REGISTER_WHO_AM_I;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &val, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret == 0) *id = (uint8_t)val;
    return ret;
}

int32_t HTS221Driver::read_humidity(float *humidity)
{
    int32_t ret;
    int16_t H0_T0_out, H1_T0_out, H_T_out, H0_rh, H1_rh;
    char reg, buf[2];
    float tmp_f;

    /* Read H0_RH_X2 */
    reg = HTS221_REGISTER_H0_RH_X2 | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    H0_rh = buf[0] >> 1;
    H1_rh = buf[1] >> 1;

    /* Read H0_T0_OUT_L */
    reg = HTS221_REGISTER_H0_T0_OUT_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    H0_T0_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

    /* Read H1_T0_OUT_L */
    reg = HTS221_REGISTER_H1_T0_OUT_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    H1_T0_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

    /* Read HUMIDITY_OUT_L */
    reg = HTS221_REGISTER_HUMIDITY_OUT_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    H_T_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

    tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
    tmp_f *= 10.0f;

    tmp_f = ( tmp_f > 1000.0f ) ? 1000.0f
        : ( tmp_f <    0.0f ) ?    0.0f
        : tmp_f;

    *humidity = tmp_f / 10.0f;

    return ret;
}

int32_t HTS221Driver::read_temperature(float *temperature)
{
    int32_t ret;
    int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
    int16_t T0_degC, T1_degC;
    char reg, buf[4], tmp;

    /* Read T0_DEGC_X8 */
    reg = HTS221_REGISTER_T0_DEGC_X8 | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    /* Read T0_T1_DEGC_H2 */
    reg = HTS221_REGISTER_T0_T1_DEGC_H2;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, &tmp, 1);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buf[0]);
    T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buf[1]);
    T0_degC = T0_degC_x8_u16 >> 3;
    T1_degC = T1_degC_x8_u16 >> 3;

    /* Read T0_OUT_L */
    reg = HTS221_REGISTER_T0_OUT_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 4);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    T0_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];
    T1_out = (((uint16_t)buf[3]) << 8) | (uint16_t)buf[2];

    /* Read T0_OUT_L */
    reg = HTS221_REGISTER_TEMP_OUT_L | 0x80;
    ret = i2c_driver->write(dev_addr, &reg, 1);
    DEBUG_ERR_WRITE(ret, reg);
    if (ret) return ret;
    ret = i2c_driver->read(dev_addr, buf, 2);
    DEBUG_ERR_READ(ret, reg);
    if (ret) return ret;

    T_out = (((uint16_t)buf[1]) << 8) | (uint16_t)buf[0];

    *temperature = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;

    return ret;
}
