/* 
 * This file is part of RaceFlight. 
 * 
 * RaceFlight is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * 
 * RaceFlight is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 * You should have received a copy of the GNU General Public License 
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */ 
#pragma once 
       
typedef struct
{
  uint8_t Power_Mode;
  uint8_t AccOutput_DataRate;
  uint8_t Axes_Enable;
  uint8_t High_Resolution;
  uint8_t BlockData_Update;
  uint8_t Endianness;
  uint8_t AccFull_Scale;
}LSM303DLHCAcc_InitTypeDef;
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;
  uint8_t HighPassFilter_CutOff_Frequency;
  uint8_t HighPassFilter_AOI1;
  uint8_t HighPassFilter_AOI2;
}LSM303DLHCAcc_FilterConfigTypeDef;
typedef struct
{
  uint8_t Temperature_Sensor;
  uint8_t MagOutput_DataRate;
  uint8_t Working_Mode;
  uint8_t MagFull_Scale;
}LSM303DLHCMag_InitTypeDef;
#define LSM303DLHC_OK ((uint32_t) 0)
#define LSM303DLHC_FAIL ((uint32_t) 0)
#define LSM303DLHC_FLAG_TIMEOUT ((uint32_t)0x1000)
#define LSM303DLHC_LONG_TIMEOUT ((uint32_t)(10 * LSM303DLHC_FLAG_TIMEOUT))
#define LSM303DLHC_I2C I2C1
#define LSM303DLHC_I2C_CLK RCC_APB1Periph_I2C1
#define LSM303DLHC_I2C_SCK_PIN GPIO_Pin_6
#define LSM303DLHC_I2C_SCK_GPIO_PORT GPIOB
#define LSM303DLHC_I2C_SCK_GPIO_CLK RCC_AHBPeriph_GPIOB
#define LSM303DLHC_I2C_SCK_SOURCE GPIO_PinSource6
#define LSM303DLHC_I2C_SCK_AF GPIO_AF_4
#define LSM303DLHC_I2C_SDA_PIN GPIO_Pin_7
#define LSM303DLHC_I2C_SDA_GPIO_PORT GPIOB
#define LSM303DLHC_I2C_SDA_GPIO_CLK RCC_AHBPeriph_GPIOB
#define LSM303DLHC_I2C_SDA_SOURCE GPIO_PinSource7
#define LSM303DLHC_I2C_SDA_AF GPIO_AF_4
#define LSM303DLHC_DRDY_PIN GPIO_Pin_2
#define LSM303DLHC_DRDY_GPIO_PORT GPIOE
#define LSM303DLHC_DRDY_GPIO_CLK RCC_AHBPeriph_GPIOE
#define LSM303DLHC_DRDY_EXTI_LINE EXTI_Line2
#define LSM303DLHC_DRDY_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define LSM303DLHC_DRDY_EXTI_PIN_SOURCE EXTI_PinSource2
#define LSM303DLHC_DRDY_EXTI_IRQn EXTI2_TS_IRQn
#define LSM303DLHC_I2C_INT1_PIN GPIO_Pin_4
#define LSM303DLHC_I2C_INT1_GPIO_PORT GPIOE
#define LSM303DLHC_I2C_INT1_GPIO_CLK RCC_AHBPeriph_GPIOE
#define LSM303DLHC_I2C_INT1_EXTI_LINE EXTI_Line4
#define LSM303DLHC_I2C_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define LSM303DLHC_I2C_INT1_EXTI_PIN_SOURCE EXTI_PinSource4
#define LSM303DLHC_I2C_INT1_EXTI_IRQn EXTI4_IRQn
#define LSM303DLHC_I2C_INT2_PIN GPIO_Pin_5
#define LSM303DLHC_I2C_INT2_GPIO_PORT GPIOE
#define LSM303DLHC_I2C_INT2_GPIO_CLK RCC_AHBPeriph_GPIOE
#define LSM303DLHC_I2C_INT2_EXTI_LINE EXTI_Line5
#define LSM303DLHC_I2C_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define LSM303DLHC_I2C_INT2_EXTI_PIN_SOURCE EXTI_PinSource5ss
#define LSM303DLHC_I2C_INT2_EXTI_IRQn EXTI9_5_IRQn
#define LSM303DLHC_CTRL_REG1_A 0x20
#define LSM303DLHC_CTRL_REG2_A 0x21
#define LSM303DLHC_CTRL_REG3_A 0x22
#define LSM303DLHC_CTRL_REG4_A 0x23
#define LSM303DLHC_CTRL_REG5_A 0x24
#define LSM303DLHC_CTRL_REG6_A 0x25
#define LSM303DLHC_REFERENCE_A 0x26
#define LSM303DLHC_STATUS_REG_A 0x27
#define LSM303DLHC_OUT_X_L_A 0x28
#define LSM303DLHC_OUT_X_H_A 0x29
#define LSM303DLHC_OUT_Y_L_A 0x2A
#define LSM303DLHC_OUT_Y_H_A 0x2B
#define LSM303DLHC_OUT_Z_L_A 0x2C
#define LSM303DLHC_OUT_Z_H_A 0x2D
#define LSM303DLHC_FIFO_CTRL_REG_A 0x2E
#define LSM303DLHC_FIFO_SRC_REG_A 0x2F
#define LSM303DLHC_INT1_CFG_A 0x30
#define LSM303DLHC_INT1_SOURCE_A 0x31
#define LSM303DLHC_INT1_THS_A 0x32
#define LSM303DLHC_INT1_DURATION_A 0x33
#define LSM303DLHC_INT2_CFG_A 0x34
#define LSM303DLHC_INT2_SOURCE_A 0x35
#define LSM303DLHC_INT2_THS_A 0x36
#define LSM303DLHC_INT2_DURATION_A 0x37
#define LSM303DLHC_CLICK_CFG_A 0x38
#define LSM303DLHC_CLICK_SOURCE_A 0x39
#define LSM303DLHC_CLICK_THS_A 0x3A
#define LSM303DLHC_TIME_LIMIT_A 0x3B
#define LSM303DLHC_TIME_LATENCY_A 0x3C
#define LSM303DLHC_TIME_WINDOW_A 0x3D
#define LSM303DLHC_CRA_REG_M 0x00
#define LSM303DLHC_CRB_REG_M 0x01
#define LSM303DLHC_MR_REG_M 0x02
#define LSM303DLHC_OUT_X_H_M 0x03
#define LSM303DLHC_OUT_X_L_M 0x04
#define LSM303DLHC_OUT_Z_H_M 0x05
#define LSM303DLHC_OUT_Z_L_M 0x06
#define LSM303DLHC_OUT_Y_H_M 0x07
#define LSM303DLHC_OUT_Y_L_M 0x08
#define LSM303DLHC_SR_REG_M 0x09
#define LSM303DLHC_IRA_REG_M 0x0A
#define LSM303DLHC_IRB_REG_M 0x0B
#define LSM303DLHC_IRC_REG_M 0x0C
#define LSM303DLHC_TEMP_OUT_H_M 0x31
#define LSM303DLHC_TEMP_OUT_L_M 0x32
#define ACC_I2C_ADDRESS 0x32
#define MAG_I2C_ADDRESS 0x3C
#define LSM303DLHC_NORMAL_MODE ((uint8_t)0x00)
#define LSM303DLHC_LOWPOWER_MODE ((uint8_t)0x08)
#define LSM303DLHC_ODR_1_HZ ((uint8_t)0x10)
#define LSM303DLHC_ODR_10_HZ ((uint8_t)0x20)
#define LSM303DLHC_ODR_25_HZ ((uint8_t)0x30)
#define LSM303DLHC_ODR_50_HZ ((uint8_t)0x40)
#define LSM303DLHC_ODR_100_HZ ((uint8_t)0x50)
#define LSM303DLHC_ODR_200_HZ ((uint8_t)0x60)
#define LSM303DLHC_ODR_400_HZ ((uint8_t)0x70)
#define LSM303DLHC_ODR_1620_HZ_LP ((uint8_t)0x80)
#define LSM303DLHC_ODR_1344_HZ ((uint8_t)0x90)
#define LSM303DLHC_X_ENABLE ((uint8_t)0x01)
#define LSM303DLHC_Y_ENABLE ((uint8_t)0x02)
#define LSM303DLHC_Z_ENABLE ((uint8_t)0x04)
#define LSM303DLHC_AXES_ENABLE ((uint8_t)0x07)
#define LSM303DLHC_AXES_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_HR_ENABLE ((uint8_t)0x08)
#define LSM303DLHC_HR_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_FULLSCALE_2G ((uint8_t)0x00)
#define LSM303DLHC_FULLSCALE_4G ((uint8_t)0x10)
#define LSM303DLHC_FULLSCALE_8G ((uint8_t)0x20)
#define LSM303DLHC_FULLSCALE_16G ((uint8_t)0x30)
#define LSM303DLHC_BlockUpdate_Continous ((uint8_t)0x00)
#define LSM303DLHC_BlockUpdate_Single ((uint8_t)0x80)
#define LSM303DLHC_BLE_LSB ((uint8_t)0x00)
#define LSM303DLHC_BLE_MSB ((uint8_t)0x40)
#define LSM303DLHC_BOOT_NORMALMODE ((uint8_t)0x00)
#define LSM303DLHC_BOOT_REBOOTMEMORY ((uint8_t)0x80)
#define LSM303DLHC_HPM_NORMAL_MODE_RES ((uint8_t)0x00)
#define LSM303DLHC_HPM_REF_SIGNAL ((uint8_t)0x40)
#define LSM303DLHC_HPM_NORMAL_MODE ((uint8_t)0x80)
#define LSM303DLHC_HPM_AUTORESET_INT ((uint8_t)0xC0)
#define LSM303DLHC_HPFCF_8 ((uint8_t)0x00)
#define LSM303DLHC_HPFCF_16 ((uint8_t)0x10)
#define LSM303DLHC_HPFCF_32 ((uint8_t)0x20)
#define LSM303DLHC_HPFCF_64 ((uint8_t)0x30)
#define LSM303DLHC_HIGHPASSFILTER_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_HIGHPASSFILTER_ENABLE ((uint8_t)0x08)
#define LSM303DLHC_HPF_CLICK_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_HPF_CLICK_ENABLE ((uint8_t)0x04)
#define LSM303DLHC_HPF_AOI1_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI1_ENABLE ((uint8_t)0x01)
#define LSM303DLHC_HPF_AOI2_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_HPF_AOI2_ENABLE ((uint8_t)0x02)
#define LSM303DLHC_IT1_CLICK ((uint8_t)0x80)
#define LSM303DLHC_IT1_AOI1 ((uint8_t)0x40)
#define LSM303DLHC_IT1_AOI2 ((uint8_t)0x20)
#define LSM303DLHC_IT1_DRY1 ((uint8_t)0x10)
#define LSM303DLHC_IT1_DRY2 ((uint8_t)0x08)
#define LSM303DLHC_IT1_WTM ((uint8_t)0x04)
#define LSM303DLHC_IT1_OVERRUN ((uint8_t)0x02)
#define LSM303DLHC_IT2_CLICK ((uint8_t)0x80)
#define LSM303DLHC_IT2_INT1 ((uint8_t)0x40)
#define LSM303DLHC_IT2_INT2 ((uint8_t)0x20)
#define LSM303DLHC_IT2_BOOT ((uint8_t)0x10)
#define LSM303DLHC_IT2_ACT ((uint8_t)0x08)
#define LSM303DLHC_IT2_HLACTIVE ((uint8_t)0x02)
#define LSM303DLHC_OR_COMBINATION ((uint8_t)0x00)
#define LSM303DLHC_AND_COMBINATION ((uint8_t)0x80)
#define LSM303DLHC_MOV_RECOGNITION ((uint8_t)0x40)
#define LSM303DLHC_POS_RECOGNITION ((uint8_t)0xC0)
#define LSM303DLHC_Z_HIGH ((uint8_t)0x20)
#define LSM303DLHC_Z_LOW ((uint8_t)0x10)
#define LSM303DLHC_Y_HIGH ((uint8_t)0x08)
#define LSM303DLHC_Y_LOW ((uint8_t)0x04)
#define LSM303DLHC_X_HIGH ((uint8_t)0x02)
#define LSM303DLHC_X_LOW ((uint8_t)0x01)
#define LSM303DLHC_Z_DOUBLE_CLICK ((uint8_t)0x20)
#define LSM303DLHC_Z_SINGLE_CLICK ((uint8_t)0x10)
#define LSM303DLHC_Y_DOUBLE_CLICK ((uint8_t)0x08)
#define LSM303DLHC_Y_SINGLE_CLICK ((uint8_t)0x04)
#define LSM303DLHC_X_DOUBLE_CLICK ((uint8_t)0x02)
#define LSM303DLHC_X_SINGLE_CLICK ((uint8_t)0x01)
#define LSM303DLHC_INT1INTERRUPT_DISABLE ((uint8_t)0x00)
#define LSM303DLHC_INT1INTERRUPT_ENABLE ((uint8_t)0x80)
#define LSM303DLHC_INT1INTERRUPT_LOW_EDGE ((uint8_t)0x20)
#define LSM303DLHC_INT1INTERRUPT_HIGH_EDGE ((uint8_t)0x00)
#define LSM303DLHC_ODR_0_75_HZ ((uint8_t) 0x00)
#define LSM303DLHC_ODR_1_5_HZ ((uint8_t) 0x04)
#define LSM303DLHC_ODR_3_0_HZ ((uint8_t) 0x08)
#define LSM303DLHC_ODR_7_5_HZ ((uint8_t) 0x0C)
#define LSM303DLHC_ODR_15_HZ ((uint8_t) 0x10)
#define LSM303DLHC_ODR_30_HZ ((uint8_t) 0x14)
#define LSM303DLHC_ODR_75_HZ ((uint8_t) 0x18)
#define LSM303DLHC_ODR_220_HZ ((uint8_t) 0x1C)
#define LSM303DLHC_FS_1_3_GA ((uint8_t) 0x20)
#define LSM303DLHC_FS_1_9_GA ((uint8_t) 0x40)
#define LSM303DLHC_FS_2_5_GA ((uint8_t) 0x60)
#define LSM303DLHC_FS_4_0_GA ((uint8_t) 0x80)
#define LSM303DLHC_FS_4_7_GA ((uint8_t) 0xA0)
#define LSM303DLHC_FS_5_6_GA ((uint8_t) 0xC0)
#define LSM303DLHC_FS_8_1_GA ((uint8_t) 0xE0)
#define LSM303DLHC_M_SENSITIVITY_XY_1_3Ga 1100
#define LSM303DLHC_M_SENSITIVITY_XY_1_9Ga 855
#define LSM303DLHC_M_SENSITIVITY_XY_2_5Ga 670
#define LSM303DLHC_M_SENSITIVITY_XY_4Ga 450
#define LSM303DLHC_M_SENSITIVITY_XY_4_7Ga 400
#define LSM303DLHC_M_SENSITIVITY_XY_5_6Ga 330
#define LSM303DLHC_M_SENSITIVITY_XY_8_1Ga 230
#define LSM303DLHC_M_SENSITIVITY_Z_1_3Ga 980
#define LSM303DLHC_M_SENSITIVITY_Z_1_9Ga 760
#define LSM303DLHC_M_SENSITIVITY_Z_2_5Ga 600
#define LSM303DLHC_M_SENSITIVITY_Z_4Ga 400
#define LSM303DLHC_M_SENSITIVITY_Z_4_7Ga 355
#define LSM303DLHC_M_SENSITIVITY_Z_5_6Ga 295
#define LSM303DLHC_M_SENSITIVITY_Z_8_1Ga 205
#define LSM303DLHC_CONTINUOS_CONVERSION ((uint8_t) 0x00)
#define LSM303DLHC_SINGLE_CONVERSION ((uint8_t) 0x01)
#define LSM303DLHC_SLEEP ((uint8_t) 0x02)
#define LSM303DLHC_TEMPSENSOR_ENABLE ((uint8_t) 0x80)
#define LSM303DLHC_TEMPSENSOR_DISABLE ((uint8_t) 0x00)
bool lsm303dlhcAccDetect(acc_t *acc);
