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
 */

#define FUNNYSTRING "GIANT GREEN CATFISH"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>

#include "platform.h"
#include "scheduler.h"
#include "version.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/color.h"
#include "common/typeconversion.h"

#include "drivers/system.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "drivers/serial.h"
#include "drivers/bus_i2c.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"

#include "eschex/esc_binary.h"

#include "io/escservo.h"
#include "io/esc_1wire.h"
#include "io/gps.h"
#include "io/gimbal.h"
#include "io/rc_controls.h"
#include "io/serial.h"
#include "io/ledstrip.h"
#include "io/flashfs.h"
#include "io/beeper.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "common/printf.h"

#include "serial_cli.h"
#include "include.h"

#include "drivers/exti.h"
#include "debug.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_spi_mpu6000.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_spi_mpu9250.h"
#include <ctype.h>
#include <strings.h>





uint8_t needDelay = 1;


uint8_t cliMode = 0;

extern uint16_t cycleTime; 

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort);

static serialPort_t *cliPort;

#ifdef ESC_1WIRE
static void rfCustom1Wire(char *cmdline);
static void rfCustomReply (void);
#endif

static void rfCustomRfblBind(char *cmdline);
static void cliResetDfu(char *cmdline);
static void cliAux(char *cmdline);
static void cliRxFail(char *cmdline);
static void cliAdjustmentRange(char *cmdline);
static void cliMotorMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliMotor(char *cmdline);
static void cliPlaySound(char *cmdline);
static void cliProfile(char *cmdline);
static void cliRateProfile(char *cmdline);
static void cliReboot(void);
static void cliSave(char *cmdline);
static void cliSerial(char *cmdline);

#ifdef USE_SERVOS
static void cliServo(char *cmdline);
static void cliServoMix(char *cmdline);
#endif

static void cliSet(char *cmdline);
static void cliGet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);
static void cliRxRange(char *cmdline);

#ifdef GPS
static void cliGpsPassthrough(char *cmdline);
#endif

static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);

#ifdef LED_STRIP
static void cliLed(char *cmdline);
static void cliColor(char *cmdline);
#endif

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline);
#endif

#ifdef USE_FLASHFS
static void cliFlashInfo(char *cmdline);
static void cliFlashErase(char *cmdline);
#ifdef USE_FLASH_TOOLS
static void cliFlashFill(char *cmdline);
static void cliFlashWrite(char *cmdline);
static void cliFlashRead(char *cmdline);
#endif
#endif

#define MSP_RF_CUSTOM_OUT        150    

extern bool processOutCommand(uint8_t cmdMSP);
extern void tailSerialReply(void);
extern void DoReboot(void);


static char cliBuffer[128];
static uint32_t bufferIndex = 0;

#ifndef USE_QUAD_MIXER_ONLY

static const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADXL", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4",
    "HEX6H", "PPM_TO_SERVO", "DUALCOPTER", "SINGLECOPTER",
    "ATAIL4", "CUSTOM", "CUSTOMAIRPLANE", "CUSTOMTRI", "QUADX1234", "QUADX1", "QUADX2", NULL
};
#endif


static const char * const featureNames[] = {
    "RX_PPM", "VBAT", "INFLIGHT_ACC_CAL", "RX_SERIAL", "MOTOR_STOP",
    "SERVO_TILT", "SOFTSERIAL", "GPS", "FAILSAFE",
    "SONAR", "TELEMETRY", "CURRENT_METER", "3D", "RX_PARALLEL_PWM",
    "RX_MSP", "RSSI_ADC", "LED_STRIP", "DISPLAY", "ONESHOT125",
    "BLACKBOX", "CHANNEL_FORWARDING", "MULTISHOT", "USE_PWM_RATE",
	"RESERVED", "TX_STYLE_EXPO", "SBUS_INVERTER", NULL
};


static const char rxFailsafeModeCharacters[] = "ahs";

static const rxFailsafeChannelMode_e rxFailsafeModesTable[RX_FAILSAFE_TYPE_COUNT][RX_FAILSAFE_MODE_COUNT] = {
    { RX_FAILSAFE_MODE_AUTO, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_INVALID },
    { RX_FAILSAFE_MODE_INVALID, RX_FAILSAFE_MODE_HOLD, RX_FAILSAFE_MODE_SET }
};

#ifndef CJMCU

static const char * const sensorTypeNames[] = {
    "GYRO", "ACC", "BARO", "MAG", "SONAR", "GPS", "GPS+MAG", NULL
};

#define SENSOR_NAMES_MASK (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

static const char * const sensorHardwareNames[4][12] = {
    { "", "None", "MPU6050", "L3G4200D", "MPU3050", "L3GD20", "MPU6000", "MPU6500", "MPU9250", "FAKE", NULL },
    { "", "None", "ADXL345", "MPU6050", "MMA845x", "BMA280", "LSM303DLHC", "MPU6000", "MPU6500", "MPU9250", "FAKE", NULL },
    { "", "None", "BMP085", "MS5611", "BMP280", NULL },
    { "", "None", "HMC5883", "AK8975", "AK8963", "MPU9250", NULL }
};
#endif

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

#ifndef SKIP_CLI_COMMAND_HELP
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name , \
    description , \
    args , \
    method \
}
#else
#define CLI_COMMAND_DEF(name, description, args, method) \
{ \
    name, \
    method \
}
#endif


const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("resetDfu", "Restart into DFU mode", NULL, cliResetDfu),
    CLI_COMMAND_DEF("adjrange", "configure adjustment ranges", NULL, cliAdjustmentRange),
    CLI_COMMAND_DEF("aux", "configure modes", NULL, cliAux),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("color", "configure colors", NULL, cliColor),
#endif
    CLI_COMMAND_DEF("defaults", "reset to defaults and reboot", NULL, cliDefaults),
    CLI_COMMAND_DEF("dump", "dump configuration",
        "[master|profile]", cliDump),
    CLI_COMMAND_DEF("exit", NULL, NULL, cliExit),
    CLI_COMMAND_DEF("feature", "configure features",
        "list\r\n"
        "\t<+|->[name]", cliFeature),
#ifdef USE_FLASHFS
    CLI_COMMAND_DEF("flash_erase", "erase flash chip", NULL, cliFlashErase),
    CLI_COMMAND_DEF("flash_info", "show flash chip info", NULL, cliFlashInfo),
#ifdef USE_FLASH_TOOLS
    CLI_COMMAND_DEF("flash_fill", NULL, "<length> <address>", cliFlashFill),
    CLI_COMMAND_DEF("flash_read", NULL, "<length> <address>", cliFlashRead),
    CLI_COMMAND_DEF("flash_write", NULL, "<address> <message>", cliFlashWrite),
#endif
#endif
    CLI_COMMAND_DEF("get", "get variable value",
            "[name]", cliGet),
#ifdef GPS
    CLI_COMMAND_DEF("gpspassthrough", "passthrough gps to serial", NULL, cliGpsPassthrough),
#endif
    CLI_COMMAND_DEF("help", NULL, NULL, cliHelp),
#ifdef LED_STRIP
    CLI_COMMAND_DEF("led", "configure leds", NULL, cliLed),
#endif
    CLI_COMMAND_DEF("map", "configure rc channel order",
        "[<map>]", cliMap),
#ifndef USE_QUAD_MIXER_ONLY
    CLI_COMMAND_DEF("mixer", "configure mixer",
        "list\r\n"
        "\t<name>", cliMixer),
#endif
    CLI_COMMAND_DEF("mmix", "custom motor mixer", NULL, cliMotorMix),
    CLI_COMMAND_DEF("motor",  "get/set motor",
       "<index> [<value>]", cliMotor),
    CLI_COMMAND_DEF("play_sound", NULL,
        "[<index>]\r\n", cliPlaySound),
    CLI_COMMAND_DEF("profile", "change profile",
        "[<index>]", cliProfile),
    CLI_COMMAND_DEF("rateprofile", "change rate profile",
        "[<index>]", cliRateProfile),
    CLI_COMMAND_DEF("rxrange", "configure rx channel ranges", NULL, cliRxRange),
    CLI_COMMAND_DEF("rxfail", "show/set rx failsafe settings", NULL, cliRxFail),
    CLI_COMMAND_DEF("save", "save and reboot", NULL, cliSave),
    CLI_COMMAND_DEF("serial", "configure serial ports", NULL, cliSerial),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("servo", "configure servos", NULL, cliServo),
#endif
    CLI_COMMAND_DEF("set", "change setting",
        "[<name>=<value>]", cliSet),
#ifdef USE_SERVOS
    CLI_COMMAND_DEF("smix", "servo mixer",
        "<rule> <servo> <source> <rate> <speed> <min> <max> <box>\r\n"
        "\treset\r\n"
        "\tload <mixer>\r\n"
        "\treverse <servo> <source> r|n", cliServoMix),
#endif
    CLI_COMMAND_DEF("status", "show status", NULL, cliStatus),
    CLI_COMMAND_DEF("version", "show version", NULL, cliVersion),
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(clicmd_t))

static const char * const lookupTableOffOn[] = {
    "OFF", "ON"
};

static const char * const lookupTableUnit[] = {
    "IMPERIAL", "METRIC"
};

static const char * const lookupTableAlignment[] = {
    "DEFAULT",
    "CW0",
    "CW90",
    "CW180",
    "CW270",
    "CW0FLIP",
    "CW90FLIP",
    "CW180FLIP",
    "CW270FLIP"
};

#ifdef GPS
static const char * const lookupTableGPSProvider[] = {
    "NMEA", "UBLOX"
};

static const char * const lookupTableGPSSBASMode[] = {
    "AUTO", "EGNOS", "WAAS", "MSAS", "GAGAN"
};
#endif

static const char * const lookupTableCurrentSensor[] = {
    "NONE", "ADC", "VIRTUAL"
};

static const char * const lookupTableGimbalMode[] = {
    "NORMAL", "MIXTILT"
};

static const char * const lookupTableBlackboxDevice[] = {
    "SERIAL", "SPIFLASH"
};


static const char * const lookupTablePidController[] = {
	"UNUSED", "RFPIDC", "RFPIDC"
};

static const char * const lookupTableSerialRX[] = {
    "SPEK1024",
    "SPEK2048",
    "SBUS",
    "SUMD",
    "SUMH",
    "XB-B",
    "XB-B-RJ01",
    "IBUS"
};

static const char * const lookupTableRFLoopCtrl[] = {
	"L1",
	"M1",
	"M2",
	"M4",
	"M8",
    "H1",
    "H2",
    "H4",
    "H8",
	"H16",
	"H32",
	"UH1",
	"UH2",
	"UH4",
	"UH8",
	"UH16",
	"UH32"
};

static const char * const lookupTableArmMethod[] = {
	"ALWAYS_DOUBLE",
	"DOUBLE_ONCE",
	"DOUBLE_ONCE_NC",
	"ALWAYS_SINGLE",
	"ALWAYS_SINGLE_NC"
};

typedef struct lookupTableEntry_s {
    const char * const *values;
    const uint8_t valueCount;
} lookupTableEntry_t;

typedef enum {
    TABLE_OFF_ON = 0,
    TABLE_UNIT,
    TABLE_ALIGNMENT,
#ifdef GPS
    TABLE_GPS_PROVIDER,
    TABLE_GPS_SBAS_MODE,
#endif
#ifdef BLACKBOX
    TABLE_BLACKBOX_DEVICE,
#endif
    TABLE_CURRENT_SENSOR,
    TABLE_GIMBAL_MODE,
    TABLE_PID_CONTROLLER,
    TABLE_SERIAL_RX,
	TABLE_RF_LOOP_CTRL,
	TABLE_ARM_METHOD,
} lookupTableIndex_e;

static const lookupTableEntry_t lookupTables[] = {
    { lookupTableOffOn, sizeof(lookupTableOffOn) / sizeof(char *) },
    { lookupTableUnit, sizeof(lookupTableUnit) / sizeof(char *) },
    { lookupTableAlignment, sizeof(lookupTableAlignment) / sizeof(char *) },
#ifdef GPS
    { lookupTableGPSProvider, sizeof(lookupTableGPSProvider) / sizeof(char *) },
    { lookupTableGPSSBASMode, sizeof(lookupTableGPSSBASMode) / sizeof(char *) },
#endif
#ifdef BLACKBOX
    { lookupTableBlackboxDevice, sizeof(lookupTableBlackboxDevice) / sizeof(char *) },
#endif
    { lookupTableCurrentSensor, sizeof(lookupTableCurrentSensor) / sizeof(char *) },
    { lookupTableGimbalMode, sizeof(lookupTableGimbalMode) / sizeof(char *) },
    { lookupTablePidController, sizeof(lookupTablePidController) / sizeof(char *) },
    { lookupTableSerialRX, sizeof(lookupTableSerialRX) / sizeof(char *) },
    { lookupTableRFLoopCtrl, sizeof(lookupTableRFLoopCtrl) / sizeof(char *) },
    { lookupTableArmMethod, sizeof(lookupTableArmMethod) / sizeof(char *) },
};

#define VALUE_TYPE_OFFSET 0
#define VALUE_SECTION_OFFSET 4
#define VALUE_MODE_OFFSET 6

typedef enum {
    
    VAR_UINT8 = (0 << VALUE_TYPE_OFFSET),
    VAR_INT8 = (1 << VALUE_TYPE_OFFSET),
    VAR_UINT16 = (2 << VALUE_TYPE_OFFSET),
    VAR_INT16 = (3 << VALUE_TYPE_OFFSET),
    VAR_UINT32 = (4 << VALUE_TYPE_OFFSET),
    VAR_FLOAT = (5 << VALUE_TYPE_OFFSET),

    
    MASTER_VALUE = (0 << VALUE_SECTION_OFFSET),
    PROFILE_VALUE = (1 << VALUE_SECTION_OFFSET),
    CONTROL_RATE_VALUE = (2 << VALUE_SECTION_OFFSET),

    
    MODE_DIRECT = (0 << VALUE_MODE_OFFSET),
    MODE_LOOKUP = (1 << VALUE_MODE_OFFSET)
} cliValueFlag_e;

#define VALUE_TYPE_MASK (0x0F)
#define VALUE_SECTION_MASK (0x30)
#define VALUE_MODE_MASK (0xC0)

typedef struct cliMinMaxConfig_s {
    const int32_t min;
    const int32_t max;
} cliMinMaxConfig_t;

typedef struct cliLookupTableConfig_s {
    const lookupTableIndex_e tableIndex;
} cliLookupTableConfig_t;

typedef union {
    cliLookupTableConfig_t lookup;
    cliMinMaxConfig_t minmax;

} cliValueConfig_t;

typedef struct {
    const char *name;
    const uint8_t type; 
    void *ptr;
    const cliValueConfig_t config;
} clivalue_t;

const clivalue_t valueTable[] = {

	{ "mid_rc",                     VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.midrc, .config.minmax = { 1200,  1700 } },
    { "min_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.mincheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_check",                  VAR_UINT16 | MASTER_VALUE,  &masterConfig.rxConfig.maxcheck, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "rssi_channel",               VAR_INT8   | MASTER_VALUE,  &masterConfig.rxConfig.rssi_channel, .config.minmax = { 0,  MAX_SUPPORTED_RC_CHANNEL_COUNT } },
    { "rssi_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.rssi_scale, .config.minmax = { RSSI_SCALE_MIN,  RSSI_SCALE_MAX } },
    { "rssi_ppm_invert",            VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.rssi_ppm_invert, .config.lookup = { TABLE_OFF_ON } },
    { "input_filtering_mode",       VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.inputFilteringMode, .config.lookup = { TABLE_OFF_ON } },
    { "rc_smoothing",               VAR_INT8   | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.rcSmoothing, .config.lookup = { TABLE_OFF_ON } },
    { "roll_yaw_cam_mix_degrees",   VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.fpvCamAngleDegrees, .config.minmax = { 0,  50 } },

    { "min_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.minthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "max_throttle",               VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.maxthrottle, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "min_command",                VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.mincommand, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },
    { "real_min_command",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.escAndServoConfig.realmincommand, .config.minmax = { PWM_RANGE_ZERO,  PWM_RANGE_MAX } },

#ifdef CC3D
    { "enable_buzzer_p6",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.use_buzzer_p6, .config.lookup = { TABLE_OFF_ON } },
#endif
    { "motor_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.motor_pwm_rate, .config.minmax = { 50,  32000 } },
    { "servo_pwm_rate",             VAR_UINT16 | MASTER_VALUE,  &masterConfig.servo_pwm_rate, .config.minmax = { 50,  498 } },

    { "small_angle",                VAR_UINT8  | MASTER_VALUE,  &masterConfig.small_angle, .config.minmax = { 0,  180 } },

    { "serialrx_provider",          VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rxConfig.serialrx_provider, .config.lookup = { TABLE_SERIAL_RX } },
    { "spektrum_sat_bind",          VAR_UINT8  | MASTER_VALUE,  &masterConfig.rxConfig.spektrum_sat_bind, .config.minmax = { SPEKTRUM_SAT_BIND_DISABLED,  SPEKTRUM_SAT_BIND_MAX} },

    { "telemetry_switch",           VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.telemetryConfig.telemetry_switch, .config.lookup = { TABLE_OFF_ON } },
    { "telemetry_inversion",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.telemetryConfig.telemetry_inversion, .config.lookup = { TABLE_OFF_ON } },

    { "battery_capacity",           VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.batteryCapacity, .config.minmax = { 0,  20000 } },
    { "vbat_scale",                 VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatscale, .config.minmax = { VBAT_SCALE_MIN,  VBAT_SCALE_MAX } },
    { "vbat_divider",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatdivider, .config.minmax = { VBAT_DIVIDER_MIN,  VBAT_DIVIDER_MAX } },
    { "vbat_max_cell_voltage",      VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatmaxcellvoltage, .config.minmax = { 10,  50 } },
    { "vbat_warning_cell_voltage",  VAR_UINT8  | MASTER_VALUE,  &masterConfig.batteryConfig.vbatwarningcellvoltage, .config.minmax = { 10,  50 } },
    { "current_meter_scale",        VAR_INT16  | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterScale, .config.minmax = { -10000,  10000 } },
    { "current_meter_offset",       VAR_UINT16 | MASTER_VALUE,  &masterConfig.batteryConfig.currentMeterOffset, .config.minmax = { 0,  3300 } },
    { "current_meter_type",         VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.batteryConfig.currentMeterType, .config.lookup = { TABLE_CURRENT_SENSOR } },

    { "align_gyro",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.gyro_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_acc",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.acc_align, .config.lookup = { TABLE_ALIGNMENT } },
    { "align_mag",                  VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.sensorAlignmentConfig.mag_align, .config.lookup = { TABLE_ALIGNMENT } },

    { "rf_loop_ctrl",               VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.rf_loop_ctrl, .config.lookup = { TABLE_RF_LOOP_CTRL } },
    { "arm_method",                 VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.arm_method, .config.lookup = { TABLE_ARM_METHOD } },

    { "deadband",                   VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.deadband, .config.minmax = { 0,  32 } },
    { "yaw_deadband",               VAR_UINT8  | PROFILE_VALUE, &masterConfig.profile[0].rcControlsConfig.yaw_deadband, .config.minmax = { 0,  100 } },

    { "yaw_control_direction",      VAR_INT8   | MASTER_VALUE,  &masterConfig.yaw_control_direction, .config.minmax = { -1,  1 } },


    { "ffamfs",      			    VAR_FLOAT   | MASTER_VALUE, &masterConfig.mixerConfig.foreAftMixerFixerStrength, .config.minmax = { 0.75,  1 } },
    { "yaw_motor_direction",        VAR_INT8   | MASTER_VALUE, &masterConfig.mixerConfig.yaw_motor_direction, .config.minmax = { -1,  1 } },

    { "failsafe_delay",             VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_delay, .config.minmax = { 0,  200 } },
    { "failsafe_off_delay",         VAR_UINT8  | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_off_delay, .config.minmax = { 0,  200 } },
    { "failsafe_throttle",          VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX } },
    { "failsafe_kill_switch",       VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.failsafeConfig.failsafe_kill_switch, .config.lookup = { TABLE_OFF_ON } },
    { "failsafe_throttle_low_delay",VAR_UINT16 | MASTER_VALUE,  &masterConfig.failsafeConfig.failsafe_throttle_low_delay, .config.minmax = { 0,  300 } },

    { "acc_hardware",               VAR_UINT8  | MASTER_VALUE,  &masterConfig.acc_hardware, .config.minmax = { 0,  ACC_MAX } },

	{ "emu_blackbox_device",        VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP,  &masterConfig.blackbox_device, .config.lookup = { TABLE_BLACKBOX_DEVICE } },

	{ "fpexpo",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcPitchExpo8, .config.minmax = { 0,  100 } },
    { "frexpo",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcRollExpo8, .config.minmax = { 0,  100 } },
    { "fyexpo",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rcYawExpo8, .config.minmax = { 0,  100 } },
    { "ftmid",                      VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrMid8, .config.minmax = { 0,  100 } },
    { "ftexpo",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].thrExpo8, .config.minmax = { 0,  100 } },
    { "frrate",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_ROLL], .config.minmax = { 0,  1800 } },
    { "fprate",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_PITCH], .config.minmax = { 0,  1800 } },
    { "fyrate",                     VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].rates[FD_YAW], .config.minmax = { 0,  1800 } },
    { "ctrarate",                   VAR_UINT8  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].dynThrPID, .config.minmax = { 0,  75} },
    { "wtpabreak",                  VAR_UINT16 | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].tpa_breakpoint, .config.minmax = { PWM_RANGE_MIN,  PWM_RANGE_MAX} },

    { "fpacrop",                    VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].PitchAcroPlusFactor, .config.minmax = {0, 200 } },
    { "fracrop",                    VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].RollAcroPlusFactor, .config.minmax = {0, 200 } },
    { "fyacrop",                    VAR_FLOAT  | CONTROL_RATE_VALUE, &masterConfig.controlRateProfiles[0].YawAcroPlusFactor, .config.minmax = {0, 200 } },


    { "fpkp",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[PITCH], .config.minmax = { 0,  300 } },
    { "fpki",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[PITCH], .config.minmax = { 0,  300 } },
    { "fpkd",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[PITCH], .config.minmax = { 0,  300 } },
    { "frkp",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[ROLL], .config.minmax = { 0,  300 } },
    { "frki",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[ROLL], .config.minmax = { 0,  300 } },
    { "frkd",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[ROLL], .config.minmax = { 0,  300 } },
    { "fykp",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.P_f[YAW], .config.minmax = { 0,  300 } },
    { "fyki",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.I_f[YAW], .config.minmax = { 0,  300 } },
    { "fykd",                       VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.D_f[YAW], .config.minmax = { 0,  300 } },

    { "wpgyrolpf",                  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wpgyrolpf, .config.minmax = {0, 255 } },
    { "wrgyrolpf",                  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wrgyrolpf, .config.minmax = {0, 255 } },
    { "wygyrolpf",                  VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wygyrolpf, .config.minmax = {0, 255 } },
    { "wpkdlpf",                    VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wpkdlpf, .config.minmax = {0, 255 } },
    { "wrkdlpf",                    VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wrkdlpf, .config.minmax = {0, 255 } },
    { "wykdlpf",                    VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.wykdlpf, .config.minmax = {0, 255 } },
    { "witchcraft",					VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.witchcraft, .config.minmax = { 0, 255 } },

    { "fcquick",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.fcquick, .config.minmax = {0, 32000 } },
    { "fcrap",                      VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.fcrap, .config.minmax = {0, 32000 } },
    { "fcpress",                    VAR_FLOAT  | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.fcpress, .config.minmax = {0, 32000 } },
    { "wypcut",                     VAR_UINT16 | PROFILE_VALUE, &masterConfig.profile[0].pidProfile.yaw_pterm_cut_hz, .config.minmax = {0, 16000 } },


    { "cledcnt",					VAR_UINT8  | PROFILE_VALUE, &masterConfig.led_count, .config.minmax = { 0, 24 } },
    { "cledclr",					VAR_UINT8  | PROFILE_VALUE, &masterConfig.led_color, .config.minmax = { 0, 255 } },
    { "cledm",						VAR_UINT8  | PROFILE_VALUE, &masterConfig.led_mode, .config.minmax = { 0, 255 } },

};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(clivalue_t))


typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

static void cliSetVar(const clivalue_t *var, const int_float_value_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);
static void cliPrint(const char *str);
static void cliPutch(void *port, char ch);
static void cliPrintf(const char *fmt, ...); 
static void cliWrite(uint8_t ch);




/************************************************************************************************/









char *StripSpaces(char *inString)
{
	uint16_t head = 0;
	uint16_t position = 0;
	uint8_t inQuote = 0;
	uint16_t inStringLength = strlen(inString);
    
	for (position = 0; position < inStringLength; position++)
	{
		if (inString[position] == '"')
			inQuote = inQuote ^ 1;
        
		if ((inQuote) || (inString[position] != ' '))
			inString[head++] = inString[position];
	}
    
    
	inString[head] = 0;
    
	return (inString);
}

char *CleanupString(char *inString)
{
	char last_char = ' ';
	uint16_t head = 0;
	uint16_t position = 0;
	uint16_t inStringLength = strlen(inString);
    
	for (position = 0; position < inStringLength; position++)
	{
		if ((last_char == ' ') && (inString[position] == ' ')) 
			continue;
        
		if (isalnum(inString[position]) || (inString[position] == ' ') || (inString[position] == '=') || (inString[position] == '"') || (inString[position] == '.') || (inString[position] == '-') || (inString[position] == '_'))
		{
			inString[head++] = inString[position];
			last_char = inString[position];
		}
	}
    
    
	inString[head] = 0;
    
	return (inString);
}





void setValue(uint16_t position, char *value)
{
	switch (valueTable[position].type & VALUE_TYPE_MASK) {
	case VAR_UINT8:
		*(uint8_t *)valueTable[position].ptr = atoi(value);
		break;

	case VAR_INT8:
		*(int8_t *)valueTable[position].ptr = atoi(value);
		break;

	case VAR_UINT16:
		*(uint16_t *)valueTable[position].ptr = atoi(value);
		break;

	case VAR_INT16:
		*(int16_t *)valueTable[position].ptr = atoi(value);
		break;

	case VAR_UINT32:
		*(uint32_t *)valueTable[position].ptr = atoi(value);
		break;

	case VAR_FLOAT:
		*(float *)valueTable[position].ptr = atof(value);
		break;
	}


}

void SetVariable(char *inString) {
	uint16_t x;
	uint16_t inStringLength;
	char *args = NULL;
	StripSpaces(inString);
    
	inStringLength = strlen(inString);
    
	for (x = 0; x < inStringLength; x++) {
		if (inString[x] == '=')
			break;
	}
    
	if (inStringLength > x) {
		args = inString + x + 1;
	}
    
	inString[x] = 0;
    
	for (x = 0; x < strlen(inString); x++)
		inString[x] = tolower(inString[x]);


	for (x = 0; x < VALUE_COUNT; x++) {
		if (!strcmp(valueTable[x].name, inString))
		{
			setValue(x, args);

		}

/*
           val = &valueTable[i];
           cliPrintf("%s = ", valueTable[i].name);
           cliPrintVar(val, len); 
           cliPrint("\r\n");
		   */
	}



}



/**********************************************************************************************************/
void OutputVar(uint16_t position)
{
	char fString[20];
	
	bzero(rf_custom_out_buffer, 255);
	switch (valueTable[position].type & VALUE_TYPE_MASK) {
	case VAR_UINT8:
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%d\n", valueTable[position].name, *(uint8_t *)valueTable[position].ptr);
		break;

	case VAR_INT8:
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%d\n", valueTable[position].name, *(int8_t *)valueTable[position].ptr);
		break;

	case VAR_UINT16:
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%d\n", valueTable[position].name, *(uint16_t *)valueTable[position].ptr);
		break;

	case VAR_INT16:
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%d\n", valueTable[position].name, *(int16_t *)valueTable[position].ptr);
		break;

	case VAR_UINT32:
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%lu\n", valueTable[position].name, *(uint32_t *)valueTable[position].ptr);
		break;
		
	case VAR_FLOAT:
		ftoa(*(float *)valueTable[position].ptr, fString);
		StripSpaces(fString);
		snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "%s=%s\n", valueTable[position].name, fString);
		fString[0] = fString[0];
		break;
	}

	rfCustomReply();

}
/**********************************************************************************************************************/




void ProcessCommand(char *inString)
{
	uint16_t inStringLength;
	char *args = NULL;
	uint16_t x;
    
	inString = CleanupString(inString);
    
	inStringLength = strlen(inString);
    
	for (x = 0; x < inStringLength; x++) {
		if (inString[x] == ' ')
			break;
	}
    
	if (inStringLength > x) {
		args = inString + x + 1;
	}
    
	inString[x] = 0;
    
	for (x = 0; x < strlen(inString); x++)
		inString[x] = tolower(inString[x]);

	if (!strcmp("set", inString))
	{
		SetVariable(args);
        
	}
    
	else if (!strcmp("dump", inString))
	{

		SKIP_GYRO = true;
		for (x = 0; x < VALUE_COUNT; x++) {
			OutputVar(x);
#if !defined(USE_VCP)
			delay(25);
#endif
		}
		SKIP_GYRO = false;
	}


	else if (!strcmp("save", inString))
	{
		SKIP_GYRO = true;
		writeEEPROM();
		readEEPROM();
		SKIP_GYRO = false;
	}
	else if (!strcmp("reboot", inString))
	{
		DoReboot();
	}
	else if (!strcmp("1wire", inString))
	{
		rfCustom1Wire(args);
	}
	else if (!strcmp("rfblbind", inString))
	{
		rfCustomRfblBind(args);
	}
	else if (!strcmp("resetdfu", inString))
	{
		systemResetToDFUloader();
	}

}










/********************************************************************************************************/





static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static void cliShowParseError(void)
{
    cliPrint("I can't figure this out. I'ma throw a parse error\r\n");
}

static void cliShowArgumentRangeError(char *name, int min, int max)
{
    cliPrintf("%s must be between %d and %d. Duh!\r\n", name, min, max);
}

static char *processChannelRangeArgs(char *ptr, channelRange_t *range, uint8_t *validArgumentCount)
{
    int val;

    for (int argIndex = 0; argIndex < 2; argIndex++) {
        ptr = strchr(ptr, ' ');
        if (ptr) {
            val = atoi(++ptr);
            val = CHANNEL_VALUE_TO_STEP(val);
            if (val >= MIN_MODE_RANGE_STEP && val <= MAX_MODE_RANGE_STEP) {
                if (argIndex == 0) {
                    range->startStep = val;
                } else {
                    range->endStep = val;
                }
                (*validArgumentCount)++;
            }
        }
    }

    return ptr;
}


static bool isEmpty(const char *string)
{
    return *string == '\0';
}

static void cliRxFail(char *cmdline)
{
    uint8_t channel;
    char buf[3];

    if (isEmpty(cmdline)) {
        
        for (channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
            cliRxFail(itoa(channel, buf, 10));
        }
    } else {
        char *ptr = cmdline;
        channel = atoi(ptr++);
        if ((channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)) {

            rxFailsafeChannelConfiguration_t *channelFailsafeConfiguration = &masterConfig.rxConfig.failsafe_channel_configurations[channel];

            uint16_t value;
            rxFailsafeChannelType_e type = (channel < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_TYPE_FLIGHT : RX_FAILSAFE_TYPE_AUX;
            rxFailsafeChannelMode_e mode = channelFailsafeConfiguration->mode;
            bool requireValue = channelFailsafeConfiguration->mode == RX_FAILSAFE_MODE_SET;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                char *p = strchr(rxFailsafeModeCharacters, *(++ptr));
                if (p) {
                    uint8_t requestedMode = p - rxFailsafeModeCharacters;
                    mode = rxFailsafeModesTable[type][requestedMode];
                } else {
                    mode = RX_FAILSAFE_MODE_INVALID;
                }
                if (mode == RX_FAILSAFE_MODE_INVALID) {
                    cliShowParseError();
                    return;
                }

                requireValue = mode == RX_FAILSAFE_MODE_SET;

                ptr = strchr(ptr, ' ');
                if (ptr) {
                    if (!requireValue) {
                        cliShowParseError();
                        return;
                    }
                    value = atoi(++ptr);
                    value = CHANNEL_VALUE_TO_RXFAIL_STEP(value);
                    if (value > MAX_RXFAIL_RANGE_STEP) {
                        cliPrint("The value you chose is out of range. This makes me a sad panda. :( \r\n");
                        return;
                    }

                    channelFailsafeConfiguration->step = value;
                } else if (requireValue) {
                    cliShowParseError();
                    return;
                }
                channelFailsafeConfiguration->mode = mode;

            }

            char modeCharacter = rxFailsafeModeCharacters[channelFailsafeConfiguration->mode];

            
            
            
            

            if (requireValue) {
                cliPrintf("rxfail %u %c %d\r\n",
                    channel,
                    modeCharacter,
                    RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfiguration->step)
                );
            } else {
                cliPrintf("rxfail %u %c\r\n",
                    channel,
                    modeCharacter
                );
            }
        } else {
            cliShowArgumentRangeError("channel", 0, MAX_SUPPORTED_RC_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef ESC_1WIRE
const oneWireParameter_t* oneWireParameters[] = {
    &motorBeaconDelayParameter,
    &motorBeaconStrengthParameter,
    &motorBeepStrengthParameter,
    &motorBrakeOnStopParameter,
    &motorDemagParameter,
    &motorDirectionParameter,
    &motorEnableTxParameter,
    &motorFrequencyParameter,
    &motorMaxThrottleParameter,
    &motorMinThrottleParameter,
    &motorStartupPowerParameter,
    &motorTimingParameter,
    &motorTempProtectionParameter,
    NULL
};

static uint8_t checkCommand(char *cmdline, const char *command) {
    if (strncasecmp(command, cmdline, strlen(command)) == 0 &&
            (cmdline[strlen(command)] == ' ' || cmdline[strlen(command)] == '\0')) {
        return true;
    }

    return false;
}

static const char* oneWireParameterValueToName(const oneWireParameterValue_t *valuesList, uint8_t value) {
    while (valuesList->name) {
        if (value == valuesList->value) {
            return valuesList->name;
        }
        valuesList++;
    }

    return "NOT FOUND";
}

static int16_t oneWireParameterNameToValue(const oneWireParameterValue_t *valuesList, const char *name) {
    while (valuesList->name) {
        if (strncasecmp(valuesList->name, name, strlen(valuesList->name)) == 0) {
            return valuesList->value;
        }
        valuesList++;
    }

    return -1;
}

int16_t oneWireParameterValueToNumber(const oneWireParameterNumerical_t *numerical, uint8_t value) {
    return numerical->step * ((int16_t)value) + numerical->offset;
}

uint8_t oneWireParameterNumberToValue(const oneWireParameterNumerical_t *numerical, int16_t value) {
    return (uint8_t)((value - numerical->offset) / numerical->step);
}


static void oneWireCommandParameterRun(const oneWireParameter_t *parameter, uint8_t motorMask, char *extraArgs) {
    int8_t escIndex;
    int16_t value;
    int16_t bytesWritten = 0;

    if (extraArgs) {
        
        if (parameter->parameterNamed) {
            value = oneWireParameterNameToValue(parameter->parameterNamed, extraArgs);
        } else {
            int16_t argValue = atoi(extraArgs);
            value = oneWireParameterNumberToValue(parameter->parameterNumerical, argValue);
        }

        if (value < 0) {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unknown parameter value\n");
            rfCustomReply();
            return;
        }

        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING WRITE %s=%s\n", parameter->name, extraArgs);
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);

        
        for (escIndex = 0; (1 << escIndex) <= motorMask; escIndex++) {
            
            if (motorMask & (1 << escIndex)) {
                bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%d=", escIndex+1);
                if (esc1WireSetParameter(escIndex, parameter, (uint8_t)value)) {
                    bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "OK\n");
                } else {
                    bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "ERROR\n");
                }
            }
        }
    } else {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING READ %s\n", parameter->name);
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);

        
        for (escIndex = 0; (1 << escIndex) <= motorMask; escIndex++) {
            
            if (motorMask & (1 << escIndex)) {
                bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%d=", escIndex+1);
                
                value = esc1WireGetParameter(escIndex, parameter);

                
                if (value < 0) {
                    bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "ERROR\n");
                    continue;
                }

                
                if (parameter->parameterNamed) {
                    bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%s\n", oneWireParameterValueToName(parameter->parameterNamed, value));
                } else {
                    bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%d\n", oneWireParameterValueToNumber(parameter->parameterNumerical, value));
                }
            }
        }
    }

    rfCustomReply();
}

static void oneWireCommandStart(char *extraArgs) {
    UNUSED(extraArgs);

    int16_t bytesWritten = 0;

    
    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING START\n");
    rfCustomReply();
    bzero(rf_custom_out_buffer, 255);

    
    int16_t motorCount = esc1WireInitialize();

    for (int motorNumber = 1; motorNumber <= motorCount; motorNumber++) {
        
        bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "%d=", motorNumber);

        
        int8_t status = esc1WireCheckMotor(motorNumber - 1);
        if (status == BLHELI_NOT_STARTED) {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "ERROR=1WIRE NOT STARTED\n");
        } else if (status == BLHELI_NO_CONNECTION) {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "ERROR=FAILED TO CONNECT\n");
        } else if (status == BLHELI_BAD_PROTOCOL) {
            uint16_t signature = esc1WireGetSignature(motorNumber - 1);
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "ERROR=FAILED TO DETERMINE MCU (%04X)\n", signature);
        } else if (status == BLHELI_BAD_LAYOUT) {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "ERROR=FAILED TO SET EEPROM LAYOUT\n");
        } else if (status == BLHELI_OK) {
            uint16_t version = esc1WireGetVersion(motorNumber - 1);
            char* name = esc1WireGetMcuName(motorNumber - 1);
            uint8_t versionMajor = version >> 8;
            uint8_t versionMinor = version & 0xFF;

            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "%s - %d.%d\n", name, versionMajor, versionMinor);
        } else {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "ERROR=UNKNOWN STATUS\n");
        }
    }

    rfCustomReply();

#ifdef ESC_HEX
    char escName[16];
    uint8_t firmwareIndex;
    const uint8_t *firmware;

    
    bzero(rf_custom_out_buffer, 255);
    bytesWritten = 0;
    bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "HEXES=");

    for (firmwareIndex = 0; escHexList[firmwareIndex] != NULL; firmwareIndex++) {
        firmware = (const uint8_t*)escHexList[firmwareIndex];

        if (!esc1WireGetHexMcuName(firmware, escName, 16)) {
            continue;
        }

        
        if (bytesWritten + strlen(escName) >= RF_BUF_SIZE - 2) {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "\n");
            rfCustomReply();

            
            bzero(rf_custom_out_buffer, 255);
            bytesWritten = 0;
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "HEXES=");
        }

        bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "%s;", escName);
    }
    bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1 - bytesWritten, "\n");

    rfCustomReply();
#endif
}

static void oneWireCommandStop(char *extraArgs) {
    UNUSED(extraArgs);

    
    if (esc1WireStatus() >= 0) {
        esc1WireRelease();
    }

    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "OK\n");
    rfCustomReply();
}

static void oneWireCommandDump(uint8_t escIndex, int motorNumber, char *extraArgs) {
    UNUSED(motorNumber);
    UNUSED(extraArgs);

    int16_t len, i, bytesWritten = 0;
    uint8_t *buf;

    len = esc1WireDumpEEprom(escIndex, &buf);

    
    if (len == BLHELI_ERROR) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to read\n");
        rfCustomReply();
    } else if (len == BLHELI_BAD_PROTOCOL) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unknown ESC MCU\n");
        rfCustomReply();
    } else if (len == 0) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=No data in ESC dump\n");
        rfCustomReply();
    } else {
        
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING DUMP\n");
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);

        for (i = 0; i < len; i++) {
            bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%02X ", buf[i]);
            if (i % 0x10 == 0xF) {
                bytesWritten += snprintf(rf_custom_out_buffer + bytesWritten, RF_BUF_SIZE-1-bytesWritten, "\n");

                
                rfCustomReply();
                
                bzero(rf_custom_out_buffer, 255);
                bytesWritten = 0;
                bzero(rf_custom_out_buffer, 255);
            }
        }

        if (i % 10 != 0) {
            rfCustomReply();
        }
    }
}

static void oneWireCommandHumanDump(uint8_t escIndex, char *extraArgs) {
    UNUSED(extraArgs);

    const oneWireParameter_t *parameter;
    int16_t len, value, bytesWritten = 0;
    uint16_t idx;
    uint8_t *buf;

    
    len = esc1WireDumpEEprom(escIndex, &buf);

    
    if (len == BLHELI_ERROR) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to read\n");
        rfCustomReply();
        return;
    } else if (len == BLHELI_BAD_PROTOCOL) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unknown ESC MCU\n");
        rfCustomReply();
        return;
    } else if (len == 0) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=No data in ESC dump\n");
        rfCustomReply();
        return;
    }

    
    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING HDUMP\n");
    rfCustomReply();
    bzero(rf_custom_out_buffer, 255);

    
    for (idx = 0; oneWireParameters[idx] != NULL; idx++) {

        parameter = oneWireParameters[idx];
        bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%s=", parameter->name);

        
        if (parameter->offset < (uint32_t)len) {
            value = esc1WireParameterFromDump(escIndex, parameter, buf);

            
            if (value == 0xFF) {
                bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "NONE\n");
                continue;
            }

            
            if (parameter->parameterNamed) {
                bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%s\n", oneWireParameterValueToName(parameter->parameterNamed, value));
            } else {
                bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "%d\n", oneWireParameterValueToNumber(parameter->parameterNumerical, value));
            }
        } else {
            bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUF_SIZE-1-bytesWritten, "ERROR\n");
        }
    }

    rfCustomReply();
}

#ifdef ESC_HEX
static void oneWireCommandFlashBuiltin(uint8_t escIndex, int motorNumber, char *extraArgs) {
    char *escName;
    int8_t status;
    uint16_t flashAddr;
    uint16_t newVersion;

    const uint8_t *firmware;

    
    status = esc1WireCheckMotor(escIndex);
    if (status == BLHELI_NOT_STARTED) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to flash, ESC not initialized\n");
        rfCustomReply();
        return;
    } else if (status == BLHELI_NO_CONNECTION) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to flash, bad ESC connection\n");
        rfCustomReply();
        return;
    } else if (status == BLHELI_BAD_PROTOCOL) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to flash, bad ESC protocol\n");
        rfCustomReply();
        return;
    }

    if (extraArgs) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING FLASH=FORCE %s\n", extraArgs);
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);

        
        escName = extraArgs;
    } else {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROCESSING FLASH=BUILT-IN\n");
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);

        
        escName = esc1WireGetMcuName(escIndex);
    }

    firmware = esc1WireFindHex(escName);

    if (firmware == NULL) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to find built-in ESC hex for ESC #%d (%s)\n", motorNumber, escName);
        rfCustomReply();
        return;
    }

    newVersion = esc1WireGetHexVersion(firmware);

    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "FLASHING=%s %d.%d\n", escName, newVersion >> 8, newVersion & 0xFF);
    rfCustomReply();
    bzero(rf_custom_out_buffer, 255);

    
    if (!esc1WireInitFlash(escIndex)) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Unable to initialize flash\n");
        rfCustomReply();
        return;
    }

    
    for (flashAddr = 0x400; flashAddr < BLHELI_FIRMWARE_SIZE; flashAddr += 0x200) {
        if (!esc1WireFlash(escIndex, (uint8_t*)&firmware[flashAddr], flashAddr, 0x200)) {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "FLASH FAILED!\n");
            rfCustomReply();
            return;
        }

        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROGRESS=%d/%d\n", flashAddr - 0x200, BLHELI_BIN_SIZE);
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);
    }

    
    if (!esc1WireFlash(escIndex, (uint8_t*)&firmware[0], 0, 0x200)) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "FLASH FAILED!\n");
        rfCustomReply();
        return;
    }
    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROGRESS=%d/%d\n", BLHELI_FIRMWARE_SIZE - 0x200, BLHELI_BIN_SIZE);
    rfCustomReply();
    bzero(rf_custom_out_buffer, 255);

    
    if (!esc1WireFlash(escIndex, (uint8_t*)&firmware[0x200], 0x200, 0x200)) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "FLASH FAILED!\n");
        rfCustomReply();
        return;
    }
    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "PROGRESS=%d/%d\n", BLHELI_FIRMWARE_SIZE, BLHELI_BIN_SIZE);
    rfCustomReply();
    bzero(rf_custom_out_buffer, 255);

    
    if (!esc1WireFlash(escIndex, (uint8_t*)&firmware[BLHELI_FIRMWARE_SIZE], BLHELI_FIRMWARE_SIZE, BLHELI_BIN_SIZE - BLHELI_FIRMWARE_SIZE)) {
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "FLASH FAILED!\n");
        rfCustomReply();
        return;
    }

    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "OK\n");
    rfCustomReply();
}
#endif /* ESC_HEX */

static void rfCustom1Wire(char *command) {
    int16_t status;
    uint8_t escIndex;
    uint8_t motorMask;

    char *args = strchr(command, ' ');
    if (args) {
        args++;
    }

	bzero(rf_custom_out_buffer, 255);

    if (command == NULL || isEmpty(command)) {
        
        status = esc1WireStatus();
        if (status < 0) {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "STATUS=STOPPED\n");
        } else {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "STATUS=STARTED=%d\n", status);
        }
        rfCustomReply();
        bzero(rf_custom_out_buffer, 255);
    } else if (checkCommand(command, "start")) {
        oneWireCommandStart(args);
    } else if (checkCommand(command, "stop")) {
        oneWireCommandStop(args);
    } else {
        
        status = esc1WireStatus();
        if (status < 0) {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=1wire interface is not started\n");
            rfCustomReply();
            return;
        }

        
        if (!args) {
            snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=No motor specified\n");
            rfCustomReply();
            return;
        }
        
        
        if (strncasecmp(args, "all", 3) == 0) {
            
            motorMask = (1 << status) - 1;

            
            args = strchr(args, ' ');
            if (args) {
                args++;
            }
        } else {
            int motorNumber;

            
            motorNumber = atoi(args);
            args = strchr(args, ' ');
            if (args) {
                args++;
            }

            
            if (motorNumber <= 0) {
                snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Invalid motor specified\n");
                rfCustomReply();
                return;
            }

            escIndex = motorNumber - 1;
            motorMask = 1 << escIndex;

            

            
            
            if (checkCommand(command, "dump")) {
                oneWireCommandDump(escIndex, motorNumber, args);
                return;
            } else if (checkCommand(command, "hdump")) {
                oneWireCommandHumanDump(escIndex, args);
                return;
#ifdef ESC_HEX
            } else if (checkCommand(command, "flash")) {
                oneWireCommandFlashBuiltin(escIndex, motorNumber, args);
                return;
#endif
            }
        }

        
        
        for (escIndex = 0; (1 << escIndex) <= motorMask; escIndex++) {
            
            if ((1 << escIndex) & motorMask) {
                
                if (esc1WireCheckMotor(escIndex) != BLHELI_OK) {
                    snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Invalid ESC on motor number %d\n", escIndex + 1);
                    rfCustomReply();
                    return;
                }
            }
        }

        
        for (int i = 0; oneWireParameters[i]; i++) {
            if (checkCommand(command, oneWireParameters[i]->name)) {
                oneWireCommandParameterRun(oneWireParameters[i], motorMask, args);
                return;
            }
        }

        
        snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "ERROR=Invalid 1wire command\n");
        rfCustomReply();
        return;
    }
}

static void rfCustomReply (void) {
	processOutCommand(MSP_RF_CUSTOM_OUT);
	tailSerialReply();
	delay(2); 
}

#endif

static void rfCustomRfblBind(char *cmdline)
{

#ifdef SPEKTRUM_TELEM
	sendSpektrumBind();
#endif
	UNUSED(cmdline);
	snprintf(rf_custom_out_buffer, RF_BUF_SIZE-1, "You don't have RFBL installed. This makes me a sad panda. :(");
	rfCustomReply();
}

static void cliResetDfu(char *cmdline)
{
	UNUSED(cmdline);
	systemResetToDFUloader();
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        
        for (i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            cliPrintf("aux %u %u %u %u %u\r\n",
                i,
                mac->modeId,
                mac->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(mac->range.endStep)
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = &currentProfile->modeActivationConditions[i];
            uint8_t validArgumentCount = 0;
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < CHECKBOX_ITEM_COUNT) {
                    mac->modeId = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    mac->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = processChannelRangeArgs(ptr, &mac->range, &validArgumentCount);

            if (validArgumentCount != 4) {
                memset(mac, 0, sizeof(modeActivationCondition_t));
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_MODE_ACTIVATION_CONDITION_COUNT - 1);
        }
    }
}

static void cliSerial(char *cmdline)
{
    int i, val;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (!serialIsPortAvailable(masterConfig.serialConfig.portConfigs[i].identifier)) {
                continue;
            };
            cliPrintf("serial %d %d %ld %ld %ld %ld\r\n" ,
                masterConfig.serialConfig.portConfigs[i].identifier,
                masterConfig.serialConfig.portConfigs[i].functionMask,
                baudRates[masterConfig.serialConfig.portConfigs[i].msp_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].gps_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].telemetry_baudrateIndex],
                baudRates[masterConfig.serialConfig.portConfigs[i].blackbox_baudrateIndex]
            );
        }
        return;
    }

    serialPortConfig_t portConfig;
    memset(&portConfig, 0 , sizeof(portConfig));

    serialPortConfig_t *currentConfig;

    uint8_t validArgumentCount = 0;

    ptr = cmdline;

    val = atoi(ptr++);
    currentConfig = serialFindPortConfiguration(val);
    if (currentConfig) {
        portConfig.identifier = val;
        validArgumentCount++;
    }

    ptr = strchr(ptr, ' ');
    if (ptr) {
        val = atoi(++ptr);
        portConfig.functionMask = val & 0xFFFF;
        validArgumentCount++;
    }

    for (i = 0; i < 4; i ++) {
        ptr = strchr(ptr, ' ');
        if (!ptr) {
            break;
        }

        val = atoi(++ptr);

        uint8_t baudRateIndex = lookupBaudRateIndex(val);
        if (baudRates[baudRateIndex] != (uint32_t) val) {
            break;
        }

        switch(i) {
            case 0:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.msp_baudrateIndex = baudRateIndex;
                break;
            case 1:
                if (baudRateIndex < BAUD_9600 || baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.gps_baudrateIndex = baudRateIndex;
                break;
            case 2:
                if (baudRateIndex != BAUD_AUTO && baudRateIndex > BAUD_115200) {
                    continue;
                }
                portConfig.telemetry_baudrateIndex = baudRateIndex;
                break;
            case 3:
                if (baudRateIndex < BAUD_19200 || baudRateIndex > BAUD_250000) {
                    continue;
                }
                portConfig.blackbox_baudrateIndex = baudRateIndex;
                break;
        }

        validArgumentCount++;
    }

    if (validArgumentCount < 6) {
        cliShowParseError();
        return;
    }

    memcpy(currentConfig, &portConfig, sizeof(portConfig));

}

static void cliAdjustmentRange(char *cmdline)
{
    int i, val = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        
        for (i = 0; i < MAX_ADJUSTMENT_RANGE_COUNT; i++) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];
            cliPrintf("adjrange %u %u %u %u %u %u %u\r\n",
                i,
                ar->adjustmentIndex,
                ar->auxChannelIndex,
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.startStep),
                MODE_STEP_TO_CHANNEL_VALUE(ar->range.endStep),
                ar->adjustmentFunction,
                ar->auxSwitchChannelIndex
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr++);
        if (i < MAX_ADJUSTMENT_RANGE_COUNT) {
            adjustmentRange_t *ar = &currentProfile->adjustmentRanges[i];
            uint8_t validArgumentCount = 0;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT) {
                    ar->adjustmentIndex = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxChannelIndex = val;
                    validArgumentCount++;
                }
            }

            ptr = processChannelRangeArgs(ptr, &ar->range, &validArgumentCount);

            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < ADJUSTMENT_FUNCTION_COUNT) {
                    ar->adjustmentFunction = val;
                    validArgumentCount++;
                }
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                val = atoi(++ptr);
                if (val >= 0 && val < MAX_AUX_CHANNEL_COUNT) {
                    ar->auxSwitchChannelIndex = val;
                    validArgumentCount++;
                }
            }

            if (validArgumentCount != 6) {
                memset(ar, 0, sizeof(adjustmentRange_t));
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_ADJUSTMENT_RANGE_COUNT - 1);
        }
    }
}

static void cliMotorMix(char *cmdline)
{
#ifdef USE_QUAD_MIXER_ONLY
    UNUSED(cmdline);
#else
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    char buf[16];
    char *ptr;

    if (isEmpty(cmdline)) {
        cliPrint("Motor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (masterConfig.customMotorMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            cliPrintf("#%d:\t", i);
            cliPrintf("%s\t", ftoa(masterConfig.customMotorMixer[i].throttle, buf));
            cliPrintf("%s\t", ftoa(masterConfig.customMotorMixer[i].roll, buf));
            cliPrintf("%s\t", ftoa(masterConfig.customMotorMixer[i].pitch, buf));
            cliPrintf("%s\r\n", ftoa(masterConfig.customMotorMixer[i].yaw, buf));
        }
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        
        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++)
            masterConfig.customMotorMixer[i].throttle = 0.0f;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrint("Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i, masterConfig.customMotorMixer);
                    cliPrintf("Loaded %s\r\n", mixerNames[i]);
                    cliMotorMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); 
        if (i < MAX_SUPPORTED_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMotorMixer[i].throttle = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMotorMixer[i].roll = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMotorMixer[i].pitch = fastA2F(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                masterConfig.customMotorMixer[i].yaw = fastA2F(++ptr);
                check++;
            }
            if (check != 4) {
                cliShowParseError();
            } else {
                cliMotorMix("");
            }
        } else {
            cliShowArgumentRangeError("index", 1, MAX_SUPPORTED_MOTORS);
        }
    }
#endif
}

static void cliRxRange(char *cmdline)
{
    int i, validArgumentCount = 0;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
            rxChannelRangeConfiguration_t *channelRangeConfiguration = &masterConfig.rxConfig.channelRanges[i];
            cliPrintf("rxrange %u %u %u\r\n", i, channelRangeConfiguration->min, channelRangeConfiguration->max);
        }
    } else if (strcasecmp(cmdline, "reset") == 0) {
        resetAllRxChannelRangeConfigurations(masterConfig.rxConfig.channelRanges);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i >= 0 && i < NON_AUX_CHANNEL_COUNT) {
            int rangeMin, rangeMax;

            ptr = strchr(ptr, ' ');
            if (ptr) {
                rangeMin = atoi(++ptr);
                validArgumentCount++;
            }

            ptr = strchr(ptr, ' ');
            if (ptr) {
                rangeMax = atoi(++ptr);
                validArgumentCount++;
            }

            if (validArgumentCount != 2) {
                cliShowParseError();
            } else if (rangeMin < PWM_PULSE_MIN || rangeMin > PWM_PULSE_MAX || rangeMax < PWM_PULSE_MIN || rangeMax > PWM_PULSE_MAX) {
                cliShowParseError();
            } else {
                rxChannelRangeConfiguration_t *channelRangeConfiguration = &masterConfig.rxConfig.channelRanges[i];
                channelRangeConfiguration->min = rangeMin;
                channelRangeConfiguration->max = rangeMax;
            }
        } else {
            cliShowArgumentRangeError("channel", 0, NON_AUX_CHANNEL_COUNT - 1);
        }
    }
}

#ifdef LED_STRIP
static void cliLed(char *cmdline)
{
    int i;
    char *ptr;
    char ledConfigBuffer[20];

    if (isEmpty(cmdline)) {
        for (i = 0; i < MAX_LED_STRIP_LENGTH; i++) {
            generateLedConfig(i, ledConfigBuffer, sizeof(ledConfigBuffer));
            cliPrintf("led %u %s\r\n", i, ledConfigBuffer);
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < MAX_LED_STRIP_LENGTH) {
            ptr = strchr(cmdline, ' ');
            if (!parseLedStripConfig(i, ++ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, MAX_LED_STRIP_LENGTH - 1);
        }
    }
}

static void cliColor(char *cmdline)
{
    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        for (i = 0; i < CONFIGURABLE_COLOR_COUNT; i++) {
            cliPrintf("color %u %d,%u,%u\r\n",
                i,
                masterConfig.colors[i].h,
                masterConfig.colors[i].s,
                masterConfig.colors[i].v
            );
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CONFIGURABLE_COLOR_COUNT) {
            ptr = strchr(cmdline, ' ');
            if (!parseColor(i, ++ptr)) {
                cliShowParseError();
            }
        } else {
            cliShowArgumentRangeError("index", 0, CONFIGURABLE_COLOR_COUNT - 1);
        }
    }
}
#endif

#ifdef USE_SERVOS
static void cliServo(char *cmdline)
{
    enum { SERVO_ARGUMENT_COUNT = 8 };
    int16_t arguments[SERVO_ARGUMENT_COUNT];

    servoParam_t *servo;

    int i;
    char *ptr;

    if (isEmpty(cmdline)) {
        
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo = &currentProfile->servoConf[i];

            cliPrintf("servo %u %d %d %d %d %d %d %d\r\n",
                i,
                servo->min,
                servo->max,
                servo->middle,
                servo->angleAtMin,
                servo->angleAtMax,
                servo->rate,
                servo->forwardFromChannel
            );
        }
    } else {
        int validArgumentCount = 0;

        ptr = cmdline;

        

        
        while (*ptr) {
            if (*ptr == '-' || (*ptr >= '0' && *ptr <= '9')) {
                if (validArgumentCount >= SERVO_ARGUMENT_COUNT) {
                    cliShowParseError();
                    return;
                }

                arguments[validArgumentCount++] = atoi(ptr);

                do {
                    ptr++;
                } while (*ptr >= '0' && *ptr <= '9');
            } else if (*ptr == ' ') {
                ptr++;
            } else {
                cliShowParseError();
                return;
            }
        }

        enum {INDEX = 0, MIN, MAX, MIDDLE, ANGLE_AT_MIN, ANGLE_AT_MAX, RATE, FORWARD};

        i = arguments[INDEX];

        
        if (validArgumentCount != SERVO_ARGUMENT_COUNT || i < 0 || i >= MAX_SUPPORTED_SERVOS) {
            cliShowParseError();
            return;
        }

        servo = &currentProfile->servoConf[i];

        if (
            arguments[MIN] < PWM_PULSE_MIN || arguments[MIN] > PWM_PULSE_MAX ||
            arguments[MAX] < PWM_PULSE_MIN || arguments[MAX] > PWM_PULSE_MAX ||
            arguments[MIDDLE] < arguments[MIN] || arguments[MIDDLE] > arguments[MAX] ||
            arguments[MIN] > arguments[MAX] || arguments[MAX] < arguments[MIN] ||
            arguments[RATE] < -100 || arguments[RATE] > 100 ||
            arguments[FORWARD] >= MAX_SUPPORTED_RC_CHANNEL_COUNT ||
            arguments[ANGLE_AT_MIN] < 0 || arguments[ANGLE_AT_MIN] > 180 ||
            arguments[ANGLE_AT_MAX] < 0 || arguments[ANGLE_AT_MAX] > 180
        ) {
            cliShowParseError();
            return;
        }

        servo->min = arguments[1];
        servo->max = arguments[2];
        servo->middle = arguments[3];
        servo->angleAtMin = arguments[4];
        servo->angleAtMax = arguments[5];
        servo->rate = arguments[6];
        servo->forwardFromChannel = arguments[7];
    }
}
#endif

#ifdef USE_SERVOS
static void cliServoMix(char *cmdline)
{
    int i;
    uint8_t len;
    char *ptr;
    int args[8], check = 0;
    len = strlen(cmdline);

    if (len == 0) {

        cliPrint("Rule\tServo\tSource\tRate\tSpeed\tMin\tMax\tBox\r\n");

        for (i = 0; i < MAX_SERVO_RULES; i++) {
            if (masterConfig.customServoMixer[i].rate == 0)
                break;

            cliPrintf("#%d:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                i,
                masterConfig.customServoMixer[i].targetChannel,
                masterConfig.customServoMixer[i].inputSource,
                masterConfig.customServoMixer[i].rate,
                masterConfig.customServoMixer[i].speed,
                masterConfig.customServoMixer[i].min,
                masterConfig.customServoMixer[i].max,
                masterConfig.customServoMixer[i].box
            );
        }
        cliPrintf("\r\n");
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        
        memset(masterConfig.customServoMixer, 0, sizeof(masterConfig.customServoMixer));
        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            currentProfile->servoConf[i].reversedSources = 0;
        }
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintf("Invalid name\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    servoMixerLoadMix(i, masterConfig.customServoMixer);
                    cliPrintf("Loaded %s\r\n", mixerNames[i]);
                    cliServoMix("");
                    break;
                }
            }
        }
    } else if (strncasecmp(cmdline, "reverse", 7) == 0) {
        enum {SERVO = 0, INPUT, REVERSE, ARGS_COUNT};
        int servoIndex, inputSource;
        ptr = strchr(cmdline, ' ');

        len = strlen(ptr);
        if (len == 0) {
            cliPrintf("s");
            for (inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                cliPrintf("\ti%d", inputSource);
            cliPrintf("\r\n");

            for (servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
                cliPrintf("%d", servoIndex);
                for (inputSource = 0; inputSource < INPUT_SOURCE_COUNT; inputSource++)
                    cliPrintf("\t%s  ", (currentProfile->servoConf[servoIndex].reversedSources & (1 << inputSource)) ? "r" : "n");
                cliPrintf("\r\n");
            }
            return;
        }

        ptr = strtok(ptr, " ");
        while (ptr != NULL && check < ARGS_COUNT - 1) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr == NULL || check != ARGS_COUNT - 1) {
            cliShowParseError();
            return;
        }

        if (args[SERVO] >= 0 && args[SERVO] < MAX_SUPPORTED_SERVOS
                && args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT
                && (*ptr == 'r' || *ptr == 'n')) {
            if (*ptr == 'r')
                currentProfile->servoConf[args[SERVO]].reversedSources |= 1 << args[INPUT];
            else
                currentProfile->servoConf[args[SERVO]].reversedSources &= ~(1 << args[INPUT]);
        } else
            cliShowParseError();

        cliServoMix("reverse");
    } else {
        enum {RULE = 0, TARGET, INPUT, RATE, SPEED, MIN, MAX, BOX, ARGS_COUNT};
        ptr = strtok(cmdline, " ");
        while (ptr != NULL && check < ARGS_COUNT) {
            args[check++] = atoi(ptr);
            ptr = strtok(NULL, " ");
        }

        if (ptr != NULL || check != ARGS_COUNT) {
            cliShowParseError();
            return;
        }

        i = args[RULE];
        if (i >= 0 && i < MAX_SERVO_RULES &&
            args[TARGET] >= 0 && args[TARGET] < MAX_SUPPORTED_SERVOS &&
            args[INPUT] >= 0 && args[INPUT] < INPUT_SOURCE_COUNT &&
            args[RATE] >= -100 && args[RATE] <= 100 &&
            args[SPEED] >= 0 && args[SPEED] <= MAX_SERVO_SPEED &&
            args[MIN] >= 0 && args[MIN] <= 100 &&
            args[MAX] >= 0 && args[MAX] <= 100 && args[MIN] < args[MAX] &&
            args[BOX] >= 0 && args[BOX] <= MAX_SERVO_BOXES) {
            masterConfig.customServoMixer[i].targetChannel = args[TARGET];
            masterConfig.customServoMixer[i].inputSource = args[INPUT];
            masterConfig.customServoMixer[i].rate = args[RATE];
            masterConfig.customServoMixer[i].speed = args[SPEED];
            masterConfig.customServoMixer[i].min = args[MIN];
            masterConfig.customServoMixer[i].max = args[MAX];
            masterConfig.customServoMixer[i].box = args[BOX];
            cliServoMix("");
        } else {
            cliShowParseError();
        }
    }
}
#endif


#ifdef USE_FLASHFS

static void cliFlashInfo(char *cmdline)
{
    const flashGeometry_t *layout = flashfsGetGeometry();

    UNUSED(cmdline);

    cliPrintf("Flash sectors=%u, sectorSize=%u, pagesPerSector=%u, pageSize=%u, totalSize=%u, usedSize=%u\r\n",
            layout->sectors, layout->sectorSize, layout->pagesPerSector, layout->pageSize, layout->totalSize, flashfsGetOffset());
}

static void cliFlashErase(char *cmdline)
{
    UNUSED(cmdline);

    cliPrintf("Erasing...\r\n");
    flashfsEraseCompletely();

    while (!flashfsIsReady()) {
        delay(100);
    }

    cliPrintf("Done.\r\n");
}

#ifdef USE_FLASH_TOOLS

static void cliFlashFill(char *cmdline)
{
	(void)(cmdline);
	uint32_t i = 1;
    uint32_t address = 1;
    char *text = "Raceflight FTW!";
    const flashGeometry_t *layout = flashfsGetGeometry();

    while (i < (layout->totalSize / 1024)) {
		i++;
		flashfsSeekAbs(address * i * 1024);
		flashfsWrite((uint8_t*)text, strlen(text), true);
		flashfsFlushSync();

    }

    cliPrintf("Your Flash is now full of goodness.\r\n", strlen(text), address);

}

static void cliFlashWrite(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    char *text = strchr(cmdline, ' ');

    if (!text) {
        cliShowParseError();
    } else {
        flashfsSeekAbs(address);
        flashfsWrite((uint8_t*)text, strlen(text), true);
        flashfsFlushSync();

        cliPrintf("Wrote %u bytes at %u.\r\n", strlen(text), address);
    }
}

static void cliFlashRead(char *cmdline)
{
    uint32_t address = atoi(cmdline);
    uint32_t length;
    int i;

    uint8_t buffer[32];

    char *nextArg = strchr(cmdline, ' ');

    if (!nextArg) {
        cliShowParseError();
    } else {
        length = atoi(nextArg);

        cliPrintf("Reading %u bytes at %u:\r\n", length, address);

        while (length > 0) {
            int bytesRead;

            bytesRead = flashfsReadAbs(address, buffer, length < sizeof(buffer) ? length : sizeof(buffer));

            for (i = 0; i < bytesRead; i++) {
                cliWrite(buffer[i]);
            }

            length -= bytesRead;
            address += bytesRead;

            if (bytesRead == 0) {
                
                break;
            }
        }
        cliPrintf("\r\n");
    }
}

#endif
#endif

static void dumpValues(uint16_t valueSection)
{
    uint32_t i;
    const clivalue_t *value;
    for (i = 0; i < VALUE_COUNT; i++) {
        value = &valueTable[i];
        if ((value->type & VALUE_SECTION_MASK) != valueSection) {
            continue;
        }

        if ( !strcmp( valueTable[i].name, "giant_green_catfish") == 0 ) {
        	cliPrintf("set %s = ", valueTable[i].name);
        	cliPrintVar(value, 0);
        	cliPrint("\r\n");
        }
    }
}

typedef enum {
    DUMP_MASTER = (1 << 0),
    DUMP_PROFILE = (1 << 1),
    DUMP_CONTROL_RATE_PROFILE = (1 << 2)
} dumpFlags_e;

#define DUMP_ALL (DUMP_MASTER | DUMP_PROFILE | DUMP_CONTROL_RATE_PROFILE)

static const char* const sectionBreak = "\r\n";

#define printSectionBreak() cliPrintf((char *)sectionBreak)

static void cliDump(char *cmdline)
{
    unsigned int i;
    char buf[16];
    uint32_t mask;

#ifndef USE_QUAD_MIXER_ONLY
    float thr, roll, pitch, yaw;
#endif

    uint8_t dumpMask = DUMP_ALL;
    if (strcasecmp(cmdline, "master") == 0) {
        dumpMask = DUMP_MASTER; 
    }
    if (strcasecmp(cmdline, "profile") == 0) {
        dumpMask = DUMP_PROFILE; 
    }
    if (strcasecmp(cmdline, "rates") == 0) {
        dumpMask = DUMP_CONTROL_RATE_PROFILE; 
    }

    if (dumpMask & DUMP_MASTER) {

        cliPrint("\r\n# version\r\n");
        cliVersion(NULL);

        cliPrint("\r\n# dump master\r\n");
        cliPrint("\r\n# mixer\r\n");

#ifndef USE_QUAD_MIXER_ONLY
        cliPrintf("mixer %s\r\n", mixerNames[masterConfig.mixerMode - 1]);

        cliPrintf("mmix reset\r\n");

        for (i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            if (masterConfig.customMotorMixer[i].throttle == 0.0f)
                break;
            thr = masterConfig.customMotorMixer[i].throttle;
            roll = masterConfig.customMotorMixer[i].roll;
            pitch = masterConfig.customMotorMixer[i].pitch;
            yaw = masterConfig.customMotorMixer[i].yaw;
            cliPrintf("mmix %d", i);
            if (thr < 0)
                cliWrite(' ');
            cliPrintf("%s", ftoa(thr, buf));
            if (roll < 0)
                cliWrite(' ');
            cliPrintf("%s", ftoa(roll, buf));
            if (pitch < 0)
                cliWrite(' ');
            cliPrintf("%s", ftoa(pitch, buf));
            if (yaw < 0)
                cliWrite(' ');
            cliPrintf("%s\r\n", ftoa(yaw, buf));
        }

        
        cliPrintf("smix reset\r\n");

        for (i = 0; i < MAX_SERVO_RULES; i++) {

            if (masterConfig.customServoMixer[i].rate == 0)
                break;

            cliPrintf("smix %d %d %d %d %d %d %d %d\r\n",
                i,
                masterConfig.customServoMixer[i].targetChannel,
                masterConfig.customServoMixer[i].inputSource,
                masterConfig.customServoMixer[i].rate,
                masterConfig.customServoMixer[i].speed,
                masterConfig.customServoMixer[i].min,
                masterConfig.customServoMixer[i].max,
                masterConfig.customServoMixer[i].box
            );
        }

#endif

        cliPrint("\r\n\r\n# feature\r\n");

        mask = featureMask();
        for (i = 0; ; i++) { 
            if (featureNames[i] == NULL)
                break;
            cliPrintf("feature -%s\r\n", featureNames[i]);
        }
        for (i = 0; ; i++) {  
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf("feature %s\r\n", featureNames[i]);
        }

        cliPrint("\r\n\r\n# map\r\n");

        for (i = 0; i < 8; i++)
            buf[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
        buf[i] = '\0';
        cliPrintf("map %s\r\n", buf);

        cliPrint("\r\n\r\n# serial\r\n");
        cliSerial("");

#ifdef LED_STRIP
        cliPrint("\r\n\r\n# led\r\n");
        cliLed("");

        cliPrint("\r\n\r\n# color\r\n");
        cliColor("");
#endif
        printSectionBreak();
        dumpValues(MASTER_VALUE);

        cliPrint("\r\n# rxfail\r\n");
        cliRxFail("");
    }

    if (dumpMask & DUMP_PROFILE) {
        cliPrint("\r\n# dump profile\r\n");

        cliPrint("\r\n# profile\r\n");
        cliProfile("");

        cliPrint("\r\n# aux\r\n");

        cliAux("");

        cliPrint("\r\n# adjrange\r\n");

        cliAdjustmentRange("");

        cliPrintf("\r\n# rxrange\r\n");

        cliRxRange("");

#ifdef USE_SERVOS
        cliPrint("\r\n# servo\r\n");

        cliServo("");

        
        unsigned int channel;

        for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            for (channel = 0; channel < INPUT_SOURCE_COUNT; channel++) {
                if (servoDirection(i, channel) < 0) {
                    cliPrintf("smix reverse %d %d r\r\n", i , channel);
                }
            }
        }
#endif

        printSectionBreak();

        dumpValues(PROFILE_VALUE);
    }

    if (dumpMask & DUMP_CONTROL_RATE_PROFILE) {
        cliPrint("\r\n# dump rates\r\n");

        cliPrint("\r\n# rateprofile\r\n");
        cliRateProfile("");

        printSectionBreak();

        dumpValues(CONTROL_RATE_VALUE);
    }
}

void cliEnter(serialPort_t *serialPort)
{
#if defined (STM32F40_41xxx) || (STM32F411xE)
	resetGyro();
#endif
    cliMode = 1;
    cliPort = serialPort;
    setPrintfSerialPort(cliPort);
    cliPrint("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
    cliPrompt();
    ENABLE_ARMING_FLAG(PREVENT_ARMING);
}

static void cliExit(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("\r\nLeaving CLI mode, unsaved changes lost.\r\n");
    *cliBuffer = '\0';
    bufferIndex = 0;
    cliMode = 0;
    
    mixerResetDisarmedMotors();
    cliReboot();

    cliPort = NULL;
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0) {
        cliPrint("Enabled: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            cliPrintf("%s ", featureNames[i]);
        }
        cliPrint("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            
            remove = true;
            cmdline++; 
            len--;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrint("Invalid name\r\n");
                break;
            }

            if (strncasecmp(cmdline, featureNames[i], len) == 0) {

                mask = 1 << i;
#ifndef GPS
                if (mask & FEATURE_GPS) {
                    cliPrint("unavailable\r\n");
                    break;
                }
#endif
#ifndef SONAR
                if (mask & FEATURE_SONAR) {
                    cliPrint("unavailable\r\n");
                    break;
                }
#endif
                if (remove) {
                    featureClear(mask);
                    cliPrint("Disabled");
                } else {
                    featureSet(mask);
                    cliPrint("Enabled");
                }
                cliPrintf(" %s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

#ifdef GPS
static void cliGpsPassthrough(char *cmdline)
{
    UNUSED(cmdline);

    gpsEnablePassthrough(cliPort);
}
#endif

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    UNUSED(cmdline);

    for (i = 0; i < CMD_COUNT; i++) {
        cliPrint(cmdTable[i].name);
#ifndef SKIP_CLI_COMMAND_HELP
        if (cmdTable[i].description) {
            cliPrintf(" - %s", cmdTable[i].description);
        }
        if (cmdTable[i].args) {
            cliPrintf("\r\n\t%s", cmdTable[i].args);
        }
#endif
        cliPrint("\r\n");
    }
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper((char)cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliShowParseError();
            return;
        }
        parseRcChannels(cmdline, &masterConfig.rxConfig);
    }
    cliPrint("Map: ");
    for (i = 0; i < 8; i++)
        out[masterConfig.rxConfig.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    cliPrintf("%s\r\n", out);
}

#ifndef USE_QUAD_MIXER_ONLY
static void cliMixer(char *cmdline)
{
    int i;
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf("Mixer: %s\r\n", mixerNames[masterConfig.mixerMode - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrint("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf("%s ", mixerNames[i]);
        }
        cliPrint("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrint("Invalid name\r\n");
            return;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            masterConfig.mixerMode = i + 1;
            break;
        }
    }

    cliMixer("");
}
#endif

static void cliMotor(char *cmdline)
{
    int motor_index = 0;
    int motor_value = 0;
    int index = 0;
    char *pch = NULL;
    char *saveptr;

    if (isEmpty(cmdline)) {
        cliShowParseError();
        return;
    }

    pch = strtok_r(cmdline, " ", &saveptr);
    while (pch != NULL) {
        switch (index) {
            case 0:
                motor_index = atoi(pch);
                break;
            case 1:
                motor_value = atoi(pch);
                break;
        }
        index++;
        pch = strtok_r(NULL, " ", &saveptr);
    }

    if (motor_index < 0 || motor_index >= MAX_SUPPORTED_MOTORS) {
        cliShowArgumentRangeError("index", 0, MAX_SUPPORTED_MOTORS);
        return;
    }

    if (index == 2) {
        if (motor_value < PWM_RANGE_MIN || motor_value > PWM_RANGE_MAX) {
            cliShowArgumentRangeError("value", 1000, 2000);
            return;
        } else {
            motor_disarmed[motor_index] = motor_value;
        }
    }

    cliPrintf("motor %d: %d\r\n", motor_index, motor_disarmed[motor_index]);
}

static void cliPlaySound(char *cmdline)
{
#if FLASH_SIZE <= 64
    UNUSED(cmdline);
#else
    int i;
    const char *name;
    static int lastSoundIdx = -1;

    if (isEmpty(cmdline)) {
        i = lastSoundIdx + 1;     
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            while (true) {   
                if (++i >= beeperTableEntryCount())
                    i = 0;   
                if ((name=beeperNameForTableIndex(i)) != NULL)
                    break;   
                if (i == lastSoundIdx + 1) {     
                    cliPrintf("Error playing sound\r\n");
                    return;
                }
            }
        }
    } else {       
        i = atoi(cmdline);
        if ((name=beeperNameForTableIndex(i)) == NULL) {
            cliPrintf("No sound for index %d\r\n", i);
            return;
        }
    }
    lastSoundIdx = i;
    beeperSilence();
    cliPrintf("Playing sound %d: %s\r\n", i, name);
    beeper(beeperModeForTableIndex(i));
#endif
}

static void cliProfile(char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {
        cliPrintf("profile %d\r\n", getCurrentProfile());
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_PROFILE_COUNT) {
            masterConfig.current_profile_index = i;
            readEEPROM();
            cliProfile("");
        }
    }
}

static void cliRateProfile(char *cmdline)
{
    int i;

    if (isEmpty(cmdline)) {

        cliPrintf("rateprofile %d\r\n", getCurrentControlRateProfile());

        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i < MAX_CONTROL_RATE_PROFILE_COUNT) {
            changeControlRateProfile(i);
            cliRateProfile("");
        }
    }
}

static void cliReboot(void) {
    cliPrint("\r\nRebooting^");
    waitForSerialPortToFinishTransmitting(cliPort);
    stopMotors();
    handleOneshotFeatureChangeOnRestart();
    systemReset();
}

static void cliSave(char *cmdline)
{
    UNUSED(cmdline);

	cliPrint("Saving\r\n");
    
	writeEEPROM();
	cliPrint("Awsomness has been saved! Type exit to reboot into awsomeness.");
}

static void cliDefaults(char *cmdline)
{
    UNUSED(cmdline);

    cliPrint("Resetting to defaults");
    resetEEPROM();
    cliReboot();
}

static void cliPrint(const char *str)
{
    serialBeginWrite(cliPort);
    while (*str)
        serialWrite(cliPort, *(str++));
    serialEndWrite(cliPort);
}

static void cliPutch(void *port, char ch)
{
    serialWrite(port, ch);
}

static void cliPrintf(const char* fmt, ...)
{

	va_list va;
    va_start(va, fmt);
    
    serialBeginWrite(cliPort);
    tfp_format(cliPort, cliPutch, fmt, va);

    serialEndWrite(cliPort);

    va_end(va);
	
	if (needDelay++ &  2)
		delay(1);
}
  
static void cliWrite(uint8_t ch)
{
    serialWrite(cliPort, ch);
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;
    char buf[8];

    void *ptr = var->ptr;
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if ((var->type & VALUE_SECTION_MASK) == CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
            value = *(uint8_t *)ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)ptr;
            break;

        case VAR_FLOAT:
            cliPrintf("%s", ftoa(*(float *)ptr, buf));
            if (full && (var->type & VALUE_MODE_MASK) == MODE_DIRECT) {
                cliPrintf(" %s", ftoa((float)var->config.minmax.min, buf));
                cliPrintf(" %s", ftoa((float)var->config.minmax.max, buf));
            }
            return; 
    }

    switch(var->type & VALUE_MODE_MASK) {
        case MODE_DIRECT:
            cliPrintf("%d", value);
            if (full) {
                cliPrintf(" %d %d", var->config.minmax.min, var->config.minmax.max);
            }
            break;
        case MODE_LOOKUP:
            cliPrintf(lookupTables[var->config.lookup.tableIndex].values[value]);
            break;
    }
}

static void cliSetVar(const clivalue_t *var, const int_float_value_t value)
{

    void *ptr = var->ptr;
    if ((var->type & VALUE_SECTION_MASK) == PROFILE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(profile_t) * masterConfig.current_profile_index);
    }
    if ((var->type & VALUE_SECTION_MASK) == CONTROL_RATE_VALUE) {
        ptr = ((uint8_t *)ptr) + (sizeof(controlRateConfig_t) * getCurrentControlRateProfile());
    }

    switch (var->type & VALUE_TYPE_MASK) {
        case VAR_UINT8:
        case VAR_INT8:
            *(int8_t *)ptr = value.int_value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(int16_t *)ptr = value.int_value;
            break;

        case VAR_UINT32:
            *(uint32_t *)ptr = value.int_value;
            break;

        case VAR_FLOAT:
            *(float *)ptr = (float)value.float_value;
            break;
    }


}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrint("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); 
            cliPrint("\r\n");

        }
    } else if ((eqptr = strstr(cmdline, "=")) != NULL) {
        

        char *lastNonSpaceCharacter = eqptr;
        while (*(lastNonSpaceCharacter - 1) == ' ') {
            lastNonSpaceCharacter--;
        }
        uint8_t variableNameLength = lastNonSpaceCharacter - cmdline;

        
        eqptr++;
        while (*(eqptr) == ' ') {
            eqptr++;
        }

        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0 && variableNameLength == strlen(valueTable[i].name)) {

                bool changeValue = false;
	            int_float_value_t tmp = { 0 };
                switch (valueTable[i].type & VALUE_MODE_MASK) {
                    case MODE_DIRECT: {
                            int32_t value = 0;
                            float valuef = 0;

                            value = atoi(eqptr);
                            valuef = fastA2F(eqptr);

                            if (valuef >= valueTable[i].config.minmax.min && valuef <= valueTable[i].config.minmax.max) { 

                                if ((valueTable[i].type & VALUE_TYPE_MASK) == VAR_FLOAT)
                                    tmp.float_value = valuef;
                                else
                                    tmp.int_value = value;

                                changeValue = true;
                            }
                        }
                        break;
                    case MODE_LOOKUP: {
                            const lookupTableEntry_t *tableEntry = &lookupTables[valueTable[i].config.lookup.tableIndex];
                            bool matched = false;
                            for (uint8_t tableValueIndex = 0; tableValueIndex < tableEntry->valueCount && !matched; tableValueIndex++) {
                                matched = strcasecmp(tableEntry->values[tableValueIndex], eqptr) == 0;

                                if (matched) {
                                    tmp.int_value = tableValueIndex;
                                    changeValue = true;
                                }
                            }
                        }
                        break;
                }

                if (changeValue) {
                    cliSetVar(val, tmp);

                    cliPrintf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrint("Invalid value\r\n");
                }

                return;
            }
        }
        cliPrint("Invalid name\r\n");
    } else {
        
    	cliGet(cmdline);
    }
}

static void cliGet(char *cmdline)
{
    uint32_t i;
    const clivalue_t *val;
    int matchedCommands = 0;

    for (i = 0; i < VALUE_COUNT; i++) {
        if (strstr(valueTable[i].name, cmdline)) {
            val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, 0);
            cliPrint("\r\n");

            matchedCommands++;
        }
    }


    if (matchedCommands) {
    	return;
    }

    cliPrint("Invalid name\r\n");
}

static void cliStatus(char *cmdline)
{

    cliPrintf("Collecting System Data... \r\n");
    delay(1000);

    cliPrintf("Reticulating Splines... \r\n");
    delay(2000);

	cliPrintf("Never gonna give you up. \r\n");
    delay(500);
	cliPrintf("Never gonna let you down. \r\n");
    delay(500);
	cliPrintf("Never gonna fly around and desert you. \r\n");
    delay(2000);

    cliPrintf("CPU Clock=%s", "All of them! \r\n");
    delay(1000);
    cliPrintf("System Uptime: %d microseconds, System load: 0.000001% \r\n", micros());
    delay(1000);
    cliPrintf("System Uptime: %d milliseconds, System load: 0.001% \r\n", millis());
    delay(1000);
    cliPrintf("System Uptime: %d seconds, System load: 1% \r\n", millis() / 1000);
    delay(1000);
    cliPrintf("System Uptime: %d minutes, System load: 110% on the reactor! \r\n", millis() / 60000);
    delay(1000);
    cliPrintf("System Uptime: %d hours, System load: IT'S OVER 9000!!!! \r\n", rand() );
    delay(1000);
    cliPrintf("System Uptime: %d days, System load: ∞ \r\n", rand());
    delay(1000);

	cliPrintf("We're just jokin'. The truth is average system load means almost nothing in a real-time system like this. \r\n");
	cliPrintf("Go fly and have fun. Let RaceFlight handle the hardware for you. :) \r\n");

    if ( (!isEmpty(cmdline)) && (millis() < 1) ) { 
    	cliPrintf(FUNNYSTRING);
    }

    cliPrint("\r\n");

#ifdef USE_I2C
    
#else
    
#endif

    
}

static void cliVersion(char *cmdline)
{
    UNUSED(cmdline);
    printf("# RaceFlight %s - %s /%s %s / %s (%s)",
		FC_VERSION_STRING,
		FC_VERSION_COMMENT,
	    targetName,
        buildDate,
        buildTime,
        shortGitRevision
    );
}

void cliProcess(void)
{
    if (!cliPort) {
        return;
    }

    while (serialRxBytesWaiting(cliPort)) {
        uint8_t c = serialRead(cliPort);
        if (c == '\t' || c == '?') {
            
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            uint32_t i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex] && bufferIndex < sizeof(cliBuffer) - 2) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        cliBuffer[bufferIndex] = '\0';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrint("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrint(cmd->name);
                    cliWrite('\t');
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {   
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {                  
            
            cliPrint("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            
            cliPrint("\r\n");

            
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; 

                const clicmd_t *cmd;
                for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                    if(!strncasecmp(cliBuffer, cmd->name, strlen(cmd->name))   
                       && !isalnum((unsigned)cliBuffer[strlen(cmd->name)]))    
                        break;
                }
                if(cmd < cmdTable + CMD_COUNT)
                    cmd->func(cliBuffer + strlen(cmd->name) + 1);
                else
                    cliPrint("I have no idea what you're talking about, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            
            if (!cliMode)
                return;

            cliPrompt();
        } else if (c == 127) {
            
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; 
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}

void cliInit(serialConfig_t *serialConfig)
{
    UNUSED(serialConfig);
}
