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
       
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
#define UINT8 uint8_t
#define INT8 int8_t
#define UINT16 uint16_t
#define INT16 int16_t
#define UINT32 uint32_t
#define INT32 int32_t
#ifndef TELEMETRY_H
#define TELEMETRY_H 
#define TELE_DEVICE_NODATA (0x00)
#define TELE_DEVICE_VOLTAGE (0x01)
#define TELE_DEVICE_TEMPERATURE (0x02)
#define TELE_DEVICE_RSV_03 (0x03)
#define TELE_DEVICE_RSV_04 (0x04)
#define TELE_DEVICE_RSV_05 (0x05)
#define TELE_DEVICE_RSV_06 (0x06)
#define TELE_DEVICE_RSV_07 (0x07)
#define TELE_DEVICE_RSV_08 (0x08)
#define TELE_DEVICE_RSV_09 (0x09)
#define TELE_DEVICE_PBOX (0x0A)
#define TELE_DEVICE_RSV_0B (0x0B)
#define TELE_DEVICE_TEXTGEN (0x0C)
#define TELE_DEVICE_AIRSPEED (0x11)
#define TELE_DEVICE_ALTITUDE (0x12)
#define TELE_DEVICE_GMETER (0x14)
#define TELE_DEVICE_JETCAT (0x15)
#define TELE_DEVICE_GPS_LOC (0x16)
#define TELE_DEVICE_GPS_STATS (0x17)
#define TELE_DEVICE_RX_MAH (0x18)
#define TELE_DEVICE_JETCAT_2 (0x19)
#define TELE_DEVICE_GYRO (0x1A)
#define TELE_DEVICE_ATTMAG (0x1B)
#define TELE_DEVICE_AS3X_LEGACYGAIN (0x1F)
#define TELE_DEVICE_ESC (0x20)
#define TELE_DEVICE_FUEL (0x22)
#define TELE_DEVICE_ALPHA6 (0x24)
#define TELE_DEVICE_FP_MAH (0x34)
#define TELE_DEVICE_DIGITAL_AIR (0x36)
#define TELE_DEVICE_STRAIN (0x38)
#define TELE_DEVICE_LIPOMON (0x3A)
#define TELE_DEVICE_LIPOMON_14 (0x3F)
#define TELE_DEVICE_VARIO_S (0x40)
#define TELE_DEVICE_VARIO_TE (0x43)
#define TELE_DEVICE_USER_16SU (0x50)
#define TELE_DEVICE_USER_16SU32U (0x52)
#define TELE_DEVICE_USER_16SU32S (0x54)
#define TELE_DEVICE_USER_16U32SU (0x56)
#define TELE_DEVICE_RSV_60 (0x60)
#define TELE_DEVICE_RSV_68 (0x68)
#define TELE_DEVICE_RSV_69 (0x69)
#define TELE_DEVICE_RSV_6A (0x6A)
#define TELE_DEVICE_RSV_6B (0x6B)
#define TELE_DEVICE_RSV_6C (0x6C)
#define TELE_DEVICE_RSV_6D (0x6D)
#define TELE_DEVICE_RSV_6E (0x6E)
#define TELE_DEVICE_RSV_6F (0x6F)
#define TELE_DEVICE_RSV_70 (0x70)
#define TELE_DEVICE_FRAMEDATA (0x7D)
#define TELE_DEVICE_RPM (0x7E)
#define TELE_DEVICE_QOS (0x7F)
#define TELE_DEVICE_MAX (0x7F)
#define TELE_DEVICE_SHORTRANGE (0x80)
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 sField1,
   sField2,
   sField3;
 UINT16 uField1,
   uField2,
   uField3,
   uField4;
} STRU_TELE_USER_16SU;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 sField1,
   sField2;
 UINT16 uField1,
   uField2,
   uField3;
 UINT32 u32Field;
} STRU_TELE_USER_16SU32U;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 sField1,
   sField2;
 UINT16 uField1,
   uField2,
   uField3;
 INT32 s32Field;
} STRU_TELE_USER_16SU32S;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 uField1;
 INT32 s32Field;
 UINT32 u32Field1,
   u32Field2;
} STRU_TELE_USER_16U32SU;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT8 lineNumber;
 char text[13];
} STRU_TELE_TEXTGEN;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 volt1;
 UINT16 volt2;
 UINT16 capacity1;
 UINT16 capacity2;
 UINT16 spare16_1;
 UINT16 spare16_2;
 UINT8 spare;
 UINT8 alarms;
} STRU_TELE_POWERBOX;
#define TELE_PBOX_ALARM_VOLTAGE_1 (0x01)
#define TELE_PBOX_ALARM_VOLTAGE_2 (0x02)
#define TELE_PBOX_ALARM_CAPACITY_1 (0x04)
#define TELE_PBOX_ALARM_CAPACITY_2 (0x08)
#define TELE_PBOX_ALARM_RESERVED_1 (0x40)
#define TELE_PBOX_ALARM_RESERVED_2 (0x80)
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 current_A;
 INT16 chargeUsed_A;
 UINT16 volts_A;
 INT16 current_B;
 INT16 chargeUsed_B;
 UINT16 volts_B;
 UINT16 spare;
} STRU_TELE_RX_MAH;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 altitude;
 INT16 delta_0250ms,
    delta_0500ms,
    delta_1000ms,
    delta_1500ms,
    delta_2000ms,
    delta_3000ms;
} STRU_TELE_VARIO_S;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 iStaticPressure,
    iPitotPressure;
 UINT16 iStaticTemp,
    iPitotTemp;
 INT8 staticRate1s,
    pitotRate1s,
    staticRate3s,
    pitotRate3s;
 INT16 airSpeed;
} STRU_TELE_VARIO_TE;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 altitude;
 INT16 maxAltitude;
} STRU_TELE_ALT;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 airspeed;
 UINT16 maxAirspeed;
} STRU_TELE_SPEED;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 RPM;
 UINT16 voltsInput;
 UINT16 tempFET;
 UINT16 currentMotor;
 UINT16 tempBEC;
 UINT8 currentBEC;
 UINT8 voltsBEC;
 UINT8 throttle;
 UINT8 powerOut;
} STRU_TELE_ESC;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 fuelConsumed_A;
 UINT16 flowRate_A;
 UINT16 temp_A;
 UINT16 fuelConsumed_B;
 UINT16 flowRate_B;
 UINT16 temp_B;
 UINT16 spare;
} STRU_TELE_FUEL;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 current_A;
 INT16 chargeUsed_A;
 UINT16 temp_A;
 INT16 current_B;
 INT16 chargeUsed_B;
 UINT16 temp_B;
 UINT16 spare;
} STRU_TELE_FP_MAH;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 digital;
 UINT16 pressure;
} STRU_TELE_DIGITAL_AIR;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 strain_A,
    strain_B,
    strain_C,
    strain_D;
} STRU_TELE_STRAIN;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 cell[6];
 UINT16 temp;
} STRU_TELE_LIPOMON;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT8 cell[14];
} STRU_TELE_LIPOMON_14;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 GForceX;
 INT16 GForceY;
 INT16 GForceZ;
 INT16 maxGForceX;
 INT16 maxGForceY;
 INT16 maxGForceZ;
 INT16 minGForceZ;
} STRU_TELE_G_METER;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT8 status;
 UINT8 throttle;
 UINT16 packVoltage;
 UINT16 pumpVoltage;
 UINT32 RPM;
 UINT16 EGT;
 UINT8 offCondition;
 UINT8 spare;
} STRU_TELE_JETCAT;
enum JETCAT_ECU_TURBINE_STATE {
 JETCAT_ECU_STATE_OFF = 0,
 JETCAT_ECU_STATE_WAIT_for_RPM,
 JETCAT_ECU_STATE_Ignite,
 JETCAT_ECU_STATE_Accelerate,
 JETCAT_ECU_STATE_Stabilise,
 JETCAT_ECU_STATE_Learn_HI,
 JETCAT_ECU_STATE_Learn_LO,
 JETCAT_ECU_STATE_UNDEFINED,
 JETCAT_ECU_STATE_Slow_Down,
 JETCAT_ECU_STATE_Manual,
 JETCAT_ECU_STATE_AutoOff,
 JETCAT_ECU_STATE_Run,
 JETCAT_ECU_STATE_Accleleration_delay,
 JETCAT_ECU_STATE_SpeedReg,
 JETCAT_ECU_STATE_Two_Shaft_Regulate,
 JETCAT_ECU_STATE_PreHeat1,
 JETCAT_ECU_STATE_PreHeat2,
 JETCAT_ECU_STATE_MainFStart,
 JETCAT_ECU_STATE_NotUsed,
 JETCAT_ECU_STATE_KeroFullOn,
 EVOJET_ECU_STATE_off = 0x20,
 EVOJET_ECU_STATE_ignt = 0x21,
 EVOJET_ECU_STATE_acce = 0x22,
 EVOJET_ECU_STATE_run = 0x23,
 EVOJET_ECU_STATE_cal = 0x24,
 EVOJET_ECU_STATE_cool = 0x25,
 EVOJET_ECU_STATE_fire = 0x26,
 EVOJET_ECU_STATE_glow = 0x27,
 EVOJET_ECU_STATE_heat = 0x28,
 EVOJET_ECU_STATE_idle = 0x29,
 EVOJET_ECU_STATE_lock = 0x2A,
 EVOJET_ECU_STATE_rel = 0x2B,
 EVOJET_ECU_STATE_spin = 0x2C,
 EVOJET_ECU_STATE_stop = 0x2D,
 HORNET_ECU_STATE_OFF = 0x30,
 HORNET_ECU_STATE_SLOWDOWN = 0x31,
 HORNET_ECU_STATE_COOL_DOWN = 0x32,
 HORNET_ECU_STATE_AUTO = 0x33,
 HORNET_ECU_STATE_AUTO_HC = 0x34,
 HORNET_ECU_STATE_BURNER_ON = 0x35,
 HORNET_ECU_STATE_CAL_IDLE = 0x36,
 HORNET_ECU_STATE_CALIBRATE = 0x37,
 HORNET_ECU_STATE_DEV_DELAY = 0x38,
 HORNET_ECU_STATE_EMERGENCY = 0x39,
 HORNET_ECU_STATE_FUEL_HEAT = 0x3A,
 HORNET_ECU_STATE_FUEL_IGNITE = 0x3B,
 HORNET_ECU_STATE_GO_IDLE = 0x3C,
 HORNET_ECU_STATE_PROP_IGNITE = 0x3D,
 HORNET_ECU_STATE_RAMP_DELAY = 0x3E,
 HORNET_ECU_STATE_RAMP_UP = 0x3F,
 HORNET_ECU_STATE_STANDBY = 0x40,
 HORNET_ECU_STATE_STEADY = 0x41,
 HORNET_ECU_STATE_WAIT_ACC = 0x42,
 HORNET_ECU_STATE_ERROR = 0x43,
 XICOY_ECU_STATE_Temp_High = 0x50,
 XICOY_ECU_STATE_Trim_Low = 0x51,
 XICOY_ECU_STATE_Set_Idle = 0x52,
 XICOY_ECU_STATE_Ready = 0x53,
 XICOY_ECU_STATE_Ignition = 0x54,
 XICOY_ECU_STATE_Fuel_Ramp = 0x55,
 XICOY_ECU_STATE_Glow_Test = 0x56,
 XICOY_ECU_STATE_Running = 0x57,
 XICOY_ECU_STATE_Stop = 0x58,
 XICOY_ECU_STATE_Flameout = 0x59,
 XICOY_ECU_STATE_Speed_Low = 0x5A,
 XICOY_ECU_STATE_Cooling = 0x5B,
 XICOY_ECU_STATE_Igniter_Bad = 0x5C,
 XICOY_ECU_STATE_Starter_F = 0x5D,
 XICOY_ECU_STATE_Weak_Fuel = 0x5E,
 XICOY_ECU_STATE_Start_On = 0x5F,
 XICOY_ECU_STATE_Pre_Heat = 0x60,
 XICOY_ECU_STATE_Battery = 0x61,
 XICOY_ECU_STATE_Time_Out = 0x62,
 XICOY_ECU_STATE_Overload = 0x63,
 XICOY_ECU_STATE_Igniter_Fail = 0x64,
 XICOY_ECU_STATE_Burner_On = 0x65,
 XICOY_ECU_STATE_Starting = 0x66,
 XICOY_ECU_STATE_SwitchOver = 0x67,
 XICOY_ECU_STATE_Cal_Pump = 0x68,
 XICOY_ECU_STATE_Pump_Limit = 0x69,
 XICOY_ECU_STATE_No_Engine = 0x6A,
 XICOY_ECU_STATE_Pwr_Boost = 0x6B,
 XICOY_ECU_STATE_Run_Idle = 0x6C,
 XICOY_ECU_STATE_Run_Max = 0x6D,
 TURBINE_ECU_MAX_STATE = 0x74
};
enum JETCAT_ECU_OFF_CONDITIONS {
 JETCAT_ECU_OFF_No_Off_Condition_defined = 0,
 JETCAT_ECU_OFF_Shut_down_via_RC,
 JETCAT_ECU_OFF_Overtemperature,
 JETCAT_ECU_OFF_Ignition_timeout,
 JETCAT_ECU_OFF_Acceleration_time_out,
 JETCAT_ECU_OFF_Acceleration_too_slow,
 JETCAT_ECU_OFF_Over_RPM,
 JETCAT_ECU_OFF_Low_Rpm_Off,
 JETCAT_ECU_OFF_Low_Battery,
 JETCAT_ECU_OFF_Auto_Off,
 JETCAT_ECU_OFF_Low_temperature_Off,
 JETCAT_ECU_OFF_Hi_Temp_Off,
 JETCAT_ECU_OFF_Glow_Plug_defective,
 JETCAT_ECU_OFF_Watch_Dog_Timer,
 JETCAT_ECU_OFF_Fail_Safe_Off,
 JETCAT_ECU_OFF_Manual_Off,
 JETCAT_ECU_OFF_Power_fail,
 JETCAT_ECU_OFF_Temp_Sensor_fail,
 JETCAT_ECU_OFF_Fuel_fail,
 JETCAT_ECU_OFF_Prop_fail,
 JETCAT_ECU_OFF_2nd_Engine_fail,
 JETCAT_ECU_OFF_2nd_Engine_Diff_Too_High,
 JETCAT_ECU_OFF_2nd_Engine_No_Comm,
 JETCAT_ECU_MAX_OFF_COND
};
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 FuelFlowRateMLMin;
 UINT32 RestFuelVolumeInTankML;
} STRU_TELE_JETCAT2;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 altitudeLow;
 UINT32 latitude;
 UINT32 longitude;
 UINT16 course;
 UINT8 HDOP;
 UINT8 GPSflags;
} STRU_TELE_GPS_LOC;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 speed;
 UINT32 UTC;
 UINT8 numSats;
 UINT8 altitudeHigh;
} STRU_TELE_GPS_STAT;
#define GPS_INFO_FLAGS_IS_NORTH_BIT (0)
#define GPS_INFO_FLAGS_IS_NORTH (1 << GPS_INFO_FLAGS_IS_NORTH_BIT)
#define GPS_INFO_FLAGS_IS_EAST_BIT (1)
#define GPS_INFO_FLAGS_IS_EAST (1 << GPS_INFO_FLAGS_IS_EAST_BIT)
#define GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT (2)
#define GPS_INFO_FLAGS_LONGITUDE_GREATER_99 (1 << GPS_INFO_FLAGS_LONGITUDE_GREATER_99_BIT)
#define GPS_INFO_FLAGS_GPS_FIX_VALID_BIT (3)
#define GPS_INFO_FLAGS_GPS_FIX_VALID (1 << GPS_INFO_FLAGS_GPS_FIX_VALID_BIT)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT (4)
#define GPS_INFO_FLAGS_GPS_DATA_RECEIVED (1 << GPS_INFO_FLAGS_GPS_DATA_RECEIVED_BIT)
#define GPS_INFO_FLAGS_3D_FIX_BIT (5)
#define GPS_INFO_FLAGS_3D_FIX (1 << GPS_INFO_FLAGS_3D_FIX_BIT)
#define GPS_INFO_FLAGS_NEGATIVE_ALT_BIT (7)
#define GPS_INFO_FLAGS_NEGATIVE_ALT (1 << GPS_INFO_FLAGS_NEGATIVE_ALT_BIT)
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 gyroX;
 INT16 gyroY;
 INT16 gyroZ;
 INT16 maxGyroX;
 INT16 maxGyroY;
 INT16 maxGyroZ;
} STRU_TELE_GYRO;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 volts;
 UINT8 state_FM;
 UINT8 gainRoll,
    gainPitch,
    gainYaw;
 INT16 attRoll,
    attPitch,
    attYaw;
 UINT16 spare;
} STRU_TELE_ALPHA6;
#define GBOX_STATE_BOOT (0x00)
#define GBOX_STATE_INIT (0x01)
#define GBOX_STATE_READY (0x02)
#define GBOX_STATE_SENSORFAULT (0x03)
#define GBOX_STATE_POWERFAULT (0x04)
#define GBOX_STATE_MASK (0x0F)
#define GBOX_FMODE_FM0 (0x00)
#define GBOX_FMODE_FM1 (0x10)
#define GBOX_FMODE_FM2 (0x20)
#define GBOX_FMODE_FM3 (0x30)
#define GBOX_FMODE_FM4 (0x40)
#define GBOX_FMODE_PANIC (0x50)
#define GBOX_FMODE_MASK (0xF0)
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 INT16 attRoll;
 INT16 attPitch;
 INT16 attYaw;
 INT16 magX;
 INT16 magY;
 INT16 magZ;
} STRU_TELE_ATTMAG;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 chanData[7];
} STRU_TELE_FRAMEDATA;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 microseconds;
 UINT16 volts;
 INT16 temperature;
} STRU_TELE_RPM;
typedef struct
{
 UINT8 identifier;
 UINT8 sID;
 UINT16 A;
 UINT16 B;
 UINT16 L;
 UINT16 R;
 UINT16 F;
 UINT16 H;
 UINT16 rxVoltage;
} STRU_TELE_QOS;
typedef union
{
 UINT16 raw[8];
 STRU_TELE_QOS qos;
 STRU_TELE_RPM rpm;
 STRU_TELE_FRAMEDATA frame;
 STRU_TELE_ALT alt;
 STRU_TELE_SPEED speed;
 STRU_TELE_VARIO_TE varioTotalEnergy;
 STRU_TELE_VARIO_S varioSimple;
 STRU_TELE_G_METER accel;
 STRU_TELE_JETCAT jetcat;
 STRU_TELE_JETCAT2 jetcat2;
 STRU_TELE_GPS_LOC gpsloc;
 STRU_TELE_GPS_STAT gpsstat;
 STRU_TELE_GYRO gyro;
 STRU_TELE_ALPHA6 alpha6;
 STRU_TELE_ATTMAG attMag;
 STRU_TELE_POWERBOX powerBox;
 STRU_TELE_RX_MAH rxMAH;
 STRU_TELE_FP_MAH fpMAH;
 STRU_TELE_ESC esc;
 STRU_TELE_FUEL fuel;
 STRU_TELE_DIGITAL_AIR digAir;
 STRU_TELE_STRAIN strain;
 STRU_TELE_LIPOMON lipomon;
 STRU_TELE_LIPOMON_14 lipomon14;
 STRU_TELE_USER_16SU user_16SU;
 STRU_TELE_USER_16SU32U user_16SU32U;
 STRU_TELE_USER_16SU32S user_16SU32S;
 STRU_TELE_USER_16U32SU user_16U32SU;
 STRU_TELE_TEXTGEN user_text;
} UN_TELEMETRY;
#endif
