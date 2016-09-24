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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "debug.h"

#include "common/axis.h"

#include "rx/rx.h"
#include "drivers/system.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "flight/failsafe.h"

/*
 * Usage:
 *
 * failsafeInit() and useFailsafeConfig() must be called before the other methods are used.
 *
 * failsafeInit() and useFailsafeConfig() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

static failsafeState_t failsafeState;

static failsafeConfig_t *failsafeConfig;

static rxConfig_t *rxConfig;

static uint16_t deadband3dThrottle;           

static void failsafeReset(void)
{
    failsafeState.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + failsafeConfig->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafeState.validRxDataReceivedAt = 0;
    failsafeState.validRxDataFailedAt = 0;
    failsafeState.throttleLowPeriod = 0;
    failsafeState.landingShouldBeFinishedAt = 0;
    failsafeState.receivingRxDataPeriod = 0;
    failsafeState.receivingRxDataPeriodPreset = 0;
    failsafeState.phase = FAILSAFE_IDLE;
    failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
}

/*
 * Should called when the failsafe config needs to be changed - e.g. a different profile has been selected.
 */
void useFailsafeConfig(failsafeConfig_t *failsafeConfigToUse)
{
    failsafeConfig = failsafeConfigToUse;
    failsafeReset();
}

void failsafeInit(rxConfig_t *intialRxConfig, uint16_t deadband3d_throttle)
{
    rxConfig = intialRxConfig;

    deadband3dThrottle = deadband3d_throttle;
    failsafeState.events = 0;
    failsafeState.monitoring = false;

    return;
}

failsafePhase_e failsafePhase()
{
    return failsafeState.phase;
}

bool failsafeIsMonitoring(void)
{
    return failsafeState.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafeState.active;
}

void failsafeStartMonitoring(void)
{
    failsafeState.monitoring = true;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafeState.landingShouldBeFinishedAt);
}

static void failsafeActivate(void)
{
    failsafeState.active = true;
    failsafeState.phase = FAILSAFE_LANDING;
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    failsafeState.landingShouldBeFinishedAt = millis() + failsafeConfig->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

    failsafeState.events++;
}

static void failsafeApplyControlInput(void)
{
    for (int i = 0; i < 3; i++) {
        rcData[i] = rxConfig->midrc;
    }
    rcData[THROTTLE] = failsafeConfig->failsafe_throttle;
}

bool failsafeIsReceivingRxData(void)
{
    return (failsafeState.rxLinkState == FAILSAFE_RXLINK_UP);
}

void failsafeOnRxSuspend(uint32_t usSuspendPeriod)
{
    failsafeState.validRxDataReceivedAt += (usSuspendPeriod / 1000);    
}

void failsafeOnRxResume(void)
{
    failsafeState.validRxDataReceivedAt = millis();                     
    failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;                     
}

void failsafeOnValidDataReceived(void)
{
    failsafeState.validRxDataReceivedAt = millis();
    if ((failsafeState.validRxDataReceivedAt - failsafeState.validRxDataFailedAt) > PERIOD_RXDATA_RECOVERY) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_UP;
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafeState.validRxDataFailedAt = millis();
    if ((failsafeState.validRxDataFailedAt - failsafeState.validRxDataReceivedAt) > failsafeState.rxDataFailurePeriod) {
        failsafeState.rxLinkState = FAILSAFE_RXLINK_DOWN;
    }
}

void failsafeUpdateState(void)
{
    if (!failsafeIsMonitoring()) {
        return;
    }

    bool receivingRxData = failsafeIsReceivingRxData();        
    bool armed = ARMING_FLAG(ARMED);
    bool failsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    beeperMode_e beeperMode = BEEPER_SILENCE;

    if (!receivingRxData) {
        beeperMode = BEEPER_RX_LOST;
    }

    bool reprocessState;

    do {
        reprocessState = false;

        switch (failsafeState.phase) {
            case FAILSAFE_IDLE:
                if (armed) {
                    
                    if (THROTTLE_HIGH == calculateThrottleStatus(rxConfig, deadband3dThrottle)) {
                        failsafeState.throttleLowPeriod = millis() + failsafeConfig->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    
                    if (failsafeSwitchIsOn && failsafeConfig->failsafe_kill_switch) {
                        
                        failsafeActivate();
                        failsafeState.phase = FAILSAFE_LANDED;      
                        failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_1_SECONDS;    
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (millis() > failsafeState.throttleLowPeriod) {
                            
                            failsafeActivate();
                            failsafeState.phase = FAILSAFE_LANDED;      
                            failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; 
                        } else {
                            failsafeState.phase = FAILSAFE_RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                } else {
                    
                    if (failsafeSwitchIsOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    
                    failsafeState.throttleLowPeriod = 0;
                }
                break;

            case FAILSAFE_RX_LOSS_DETECTED:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                } else {
                    
                    failsafeActivate();
                }
                reprocessState = true;
                break;

            case FAILSAFE_LANDING:
                if (receivingRxData) {
                    failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                if (armed) {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }
                if (failsafeShouldHaveCausedLandingByNow() || !armed) {
                    failsafeState.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; 
                    failsafeState.phase = FAILSAFE_LANDED;
                    reprocessState = true;
                }
                break;

            case FAILSAFE_LANDED:
                ENABLE_ARMING_FLAG(PREVENT_ARMING); 
                mwDisarm();
                failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset; 
                failsafeState.phase = FAILSAFE_RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            case FAILSAFE_RX_LOSS_MONITORING:
                
                if (receivingRxData) {
                    if (millis() > failsafeState.receivingRxDataPeriod) {
                        
                        if (!(!isUsingSticksForArming() && IS_RC_MODE_ACTIVE(BOXARM))) {
                            DISABLE_ARMING_FLAG(PREVENT_ARMING);
                            failsafeState.phase = FAILSAFE_RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
                    }
                } else {
                    failsafeState.receivingRxDataPeriod = millis() + failsafeState.receivingRxDataPeriodPreset;
                }
                break;

            case FAILSAFE_RX_LOSS_RECOVERED:
                
                
                
                failsafeState.throttleLowPeriod = millis() + failsafeConfig->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafeState.phase = FAILSAFE_IDLE;
                failsafeState.active = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
