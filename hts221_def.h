#ifndef __HTS221_DEF_H
#define __HTS221_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Registers */
#define HTS221_REGISTER_WHO_AM_I        (uint8_t)0x0F // Device identification
#define HTS221_REGISTER_AV_CONF         (uint8_t)0x10 // Humidity and temperature average/resolution mode
#define HTS221_REGISTER_CTRL_REG1       (uint8_t)0x20
#define HTS221_REGISTER_CTRL_REG2       (uint8_t)0x21
#define HTS221_REGISTER_CTRL_REG3       (uint8_t)0x22
#define HTS221_REGISTER_STATUS_REG      (uint8_t)0x27
#define HTS221_REGISTER_HUMIDITY_OUT_L  (uint8_t)0x28
#define HTS221_REGISTER_HUMIDITY_OUT_H  (uint8_t)0x29
#define HTS221_REGISTER_TEMP_OUT_L      (uint8_t)0x2A
#define HTS221_REGISTER_TEMP_OUT_H      (uint8_t)0x2B
#define HTS221_REGISTER_H0_RH_X2        (uint8_t)0x30 /* The Registers in 0x30..0x3F address range contain calibration coefficients. */
#define HTS221_REGISTER_H1_RH_X2        (uint8_t)0x31
#define HTS221_REGISTER_T0_DEGC_X8      (uint8_t)0x32
#define HTS221_REGISTER_T1_DEGC_X8      (uint8_t)0x33
#define HTS221_REGISTER_T0_T1_DEGC_H2   (uint8_t)0x35
#define HTS221_REGISTER_H0_T0_OUT_L     (uint8_t)0x36
#define HTS221_REGISTER_H0_T0_OUT_H     (uint8_t)0x37
#define HTS221_REGISTER_H1_T0_OUT_L     (uint8_t)0x3A
#define HTS221_REGISTER_H1_T0_OUT_H     (uint8_t)0x3B
#define HTS221_REGISTER_T0_OUT_L        (uint8_t)0x3C
#define HTS221_REGISTER_T0_OUT_H        (uint8_t)0x3D
#define HTS221_REGISTER_T1_OUT_L        (uint8_t)0x3E
#define HTS221_REGISTER_T1_OUT_H        (uint8_t)0x3F

#define HTS221_BIT(x) ((uint8_t)x) /* Bitfield positioning. */

/* WHO_AM_I Register */
#define HTS221_WHO_AM_I_VALUE           (uint8_t)0xBC

/** Humidity and temperature average mode register. Default value: 0x1B
  *        7:6 Reserved.
  *        5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
  *        
  *             AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
  *          ----------------------------------------------------
  *              0    |   0   |   0   |    2
  *              0    |   0   |   1   |    4
  *              0    |   1   |   0   |    8
  *              0    |   1   |   1   |    16
  *              1    |   0   |   0   |    32
  *              1    |   0   |   1   |    64
  *              1    |   1   |   0   |    128
  *              1    |   1   |   1   |    256
  *        
  *        2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
  *             AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
  *          ------------------------------------------------------
  *              0    |   0   |   0   |    4
  *              0    |   0   |   1   |    8
  *              0    |   1   |   0   |    16
  *              0    |   1   |   1   |    32
  *              1    |   0   |   0   |    64
  *              1    |   0   |   1   |    128
  *              1    |   1   |   0   |    256
  *              1    |   1   |   1   |    512
  */

#define HTS221_AVGT_BIT           HTS221_BIT(3)
#define HTS221_AVGH_BIT           HTS221_BIT(0)
#define HTS221_AVGH_MASK          (uint8_t)0x07
#define HTS221_AVGT_MASK          (uint8_t)0x38

/** Control register 1. Default value: 0x00
  *        7 PD: power down control. 0 - power down mode; 1 - active mode.
  *        6:3 Reserved.
  *        2 BDU: block data update. 0 - continuous update
  *                                  1 - output registers not updated until MSB and LSB reading.
  *        1:0 ODR1, ODR0: output data rate selection.
  *       
  *          ODR1  | ODR0  | Humidity output data-rate(Hz)  | Pressure output data-rate(Hz)
  *          ----------------------------------------------------------------------------------
  *            0   |   0   |         one shot               |         one shot
  *            0   |   1   |            1                   |            1
  *            1   |   0   |            7                   |            7
  *            1   |   1   |           12.5                 |           12.5
  *       
  */
#define HTS221_PD_BIT           HTS221_BIT(7)
#define HTS221_BDU_BIT          HTS221_BIT(2)
#define HTS221_ODR_BIT          HTS221_BIT(0)
#define HTS221_PD_MASK          (uint8_t)0x80
#define HTS221_BDU_MASK         (uint8_t)0x04
#define HTS221_ODR_MASK         (uint8_t)0x03

/** Control register 2. Default value: 0x00
  *        7 BOOT:  Reboot memory content. 0: normal mode 
  *                                        1: reboot memory content. Self-cleared upon completation.
  *        6:2 Reserved.
  *        1 HEATHER: 0: heater enable; 1: heater disable.
  *        0 ONE_SHOT: 0: waiting for start of conversion 
  *                    1: start for a new dataset. Self-cleared upon completation.
  */
#define HTS221_BOOT_BIT         HTS221_BIT(7)
#define HTS221_HEATHER_BIT      HTS221_BIT(1)
#define HTS221_ONESHOT_BIT      HTS221_BIT(0)
#define HTS221_BOOT_MASK        (uint8_t)0x80
#define HTS221_HEATHER_MASK     (uint8_t)0x02
#define HTS221_ONE_SHOT_MASK    (uint8_t)0x01

/** Control register 3. Default value: 0x00
  *        7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
  *        6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull 
  *                                                                  1: open drain.
  *        5:3 Reserved.
  *        2 DRDY: interrupt config. 0: disable, 1: enable.
  */
#define HTS221_DRDY_H_L_BIT     HTS221_BIT(7)
#define HTS221_PP_OD_BIT        HTS221_BIT(6)
#define HTS221_DRDY_BIT         HTS221_BIT(2)
#define HTS221_DRDY_H_L_MASK    (uint8_t)0x80
#define HTS221_PP_OD_MASK       (uint8_t)0x40
#define HTS221_DRDY_MASK        (uint8_t)0x04

/**  Status register (Read Only). Default value: 0x00
  *         7:2 Reserved.
  *         1 H_DA: Humidity data available. 0: new data for humidity is not yet available 
  *                                          1: new data for humidity is available.
  *         0 T_DA: Temperature data available. 0: new data for temperature is not yet available
  *                                             1: new data for temperature is available.
  */
#define HTS221_H_DA_BIT         HTS221_BIT(1)
#define HTS221_T_DA_BIT         HTS221_BIT(0)
#define HTS221_HDA_MASK         (uint8_t)0x02
#define HTS221_TDA_MASK         (uint8_t)0x01

#ifdef __cplusplus
}
#endif

#endif /* __HTS221_DEF_H */
