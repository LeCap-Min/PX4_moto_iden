/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file commander_params.c
 *
 * Parameters definition for Commander.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Julian Oes <julian@px4.io>
 */

/**
 * Roll trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight.
 *
 * @group Radio Calibration
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);

/**
 * Pitch trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight.
 *
 * @group Radio Calibration
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);

/**
 * Yaw trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight.
 *
 * @group Radio Calibration
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * GCS connection loss time threshold
 *
 * After this amount of seconds without datalink, the GCS connection lost mode triggers
 *
 * @group Commander
 * @unit s
 * @min 5
 * @max 300
 * @decimal 1
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_T, 10);

/**
 * High Latency Datalink loss time threshold
 *
 * After this amount of seconds without datalink the data link lost mode triggers
 *
 * @group Commander
 * @unit s
 * @min 60
 * @max 3600
 */
PARAM_DEFINE_INT32(COM_HLDL_LOSS_T, 120);

/**
 * High Latency Datalink regain time threshold
 *
 * After a data link loss: after this number of seconds with a healthy datalink the 'datalink loss'
 * flag is set back to false
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 */
PARAM_DEFINE_INT32(COM_HLDL_REG_T, 0);

/**
 * Manual control loss timeout
 *
 * The time in seconds without a new setpoint from RC or Joystick, after which the connection is considered lost.
 * This must be kept short as the vehicle will use the last supplied setpoint until the timeout triggers.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 35
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);

/**
 * Home position enabled
 *
 * Set home position automatically if possible.
 *
 * @group Commander
 * @reboot_required true
 * @boolean
 */
PARAM_DEFINE_INT32(COM_HOME_EN, 1);

/**
 * Allows setting the home position after takeoff
 *
 * If set to true, the autopilot is allowed to set its home position after takeoff
 * The true home position is back-computed if a local position is estimate if available.
 * If no local position is available, home is set to the current position.
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_HOME_IN_AIR, 0);

/**
 * RC control input mode
 *
 * A value of 0 enables RC transmitter control (only). A valid RC transmitter calibration is required.
 * A value of 1 allows joystick control only. RC input handling and the associated checks are disabled.
 * A value of 2 allows either RC Transmitter or Joystick input. The first valid input is used, will fallback to other sources if the input stream becomes invalid.
 * A value of 3 allows either input from RC or joystick. The first available source is selected and used until reboot.
 * A value of 4 ignores any stick input.
 *
 * @group Commander
 * @min 0
 * @max 4
 * @value 0 RC Transmitter only
 * @value 1 Joystick only
 * @value 2 RC and Joystick with fallback
 * @value 3 RC or Joystick keep first
 * @value 4 Stick input disabled
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 3);

/**
 * RC input arm/disarm command duration
 *
 * The default value of 1000 requires the stick to be held in the arm or disarm position for 1 second.
 *
 * @group Commander
 * @min 100
 * @max 1500
 * @unit ms
 */
PARAM_DEFINE_INT32(COM_RC_ARM_HYST, 1000);

/**
 * Time-out for auto disarm after landing
 *
 * A non-zero, positive value specifies the time-out period in seconds after which the vehicle will be
 * automatically disarmed in case a landing situation has been detected during this period.
 *
 * A zero or negative value means that automatic disarming triggered by landing detection is disabled.
 *
 * @group Commander
 * @unit s
 * @decimal 1
 * @increment 0.1
 */

PARAM_DEFINE_FLOAT(COM_DISARM_LAND, 2.0f);

/**
 * Time-out for auto disarm if not taking off
 *
 * A non-zero, positive value specifies the time in seconds, within which the
 * vehicle is expected to take off after arming. In case the vehicle didn't takeoff
 * within the timeout it disarms again.
 *
 * A negative value disables autmoatic disarming triggered by a pre-takeoff timeout.
 *
 * @group Commander
 * @unit s
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_DISARM_PRFLT, 10.0f);

/**
 * GPS preflight check
 *
 * Measures taken when a check defined by EKF2_GPS_CHECK is failing.
 *
 * @group Commander
 * @value 0 Deny arming
 * @value 1 Warning only
 * @value 2 Disabled
 */
PARAM_DEFINE_INT32(COM_ARM_WO_GPS, 1);

/**
 * Arm switch is a momentary button
 *
 * 0: Arming/disarming triggers on switch transition.
 * 1: Arming/disarming triggers when holding the momentary button down
 * for COM_RC_ARM_HYST like the stick gesture.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_SWISBTN, 0);

/**
 * Allow disarming via switch/stick/button on multicopters in manual thrust modes
 *
 * 0: Disallow disarming when not landed
 * 1: Allow disarming in multicopter flight in modes where
 * the thrust is directly controlled by thr throttle stick
 * e.g. Stabilized, Acro
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_DISARM_MAN, 1);

/**
 * Battery failsafe mode
 *
 * Action the system takes at critical battery. See also BAT_CRIT_THR and BAT_EMERGEN_THR
 * for definition of battery states.
 *
 * @group Commander
 * @value 0 Warning
 * @value 2 Land mode
 * @value 3 Return at critical level, land at emergency level
 */
PARAM_DEFINE_INT32(COM_LOW_BAT_ACT, 0);

/**
 * Delay between failsafe condition triggered and failsafe reaction
 *
 * Before entering failsafe (RTL, Land, Hold), wait COM_FAIL_ACT_T seconds in Hold mode
 * for the user to realize.
 * During that time the user cannot take over control via the stick override feature (see COM_RC_OVERRIDE).
 * Afterwards the configured failsafe action is triggered and the user may use stick override.
 *
 * A zero value disables the delay and the user cannot take over via stick movements (switching modes is still allowed).
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 25.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(COM_FAIL_ACT_T, 5.f);

/**
 * Imbalanced propeller failsafe mode
 *
 * Action the system takes when an imbalanced propeller is detected by the failure detector.
 * See also FD_IMB_PROP_THR to set the failure threshold.
 *
 * @group Commander
 *
 * @value -1 Disabled
 * @value 0 Warning
 * @value 1 Return
 * @value 2 Land
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_IMB_PROP_ACT, 0);

/**
 * Time-out to wait when offboard connection is lost before triggering offboard lost action.
 *
 * See COM_OBL_RC_ACT to configure action.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 1.0f);

/**
 * Set action after a quadchute
 *
 * @value -1 Warning only
 * @value  0 Return mode
 * @value  1 Land mode
 * @value  2 Hold mode
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_QC_ACT, 0);

/**
 * Set offboard loss failsafe mode
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value 0 Position mode
 * @value 1 Altitude mode
 * @value 2 Stabilized
 * @value 3 Return mode
 * @value 4 Land mode
 * @value 5 Hold mode
 * @value 6 Terminate
 * @value 7 Disarm
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_OBL_RC_ACT, 0);

/**
 * Time-out to wait when onboard computer connection is lost before warning about loss connection.
 *
 * @group Commander
 * @unit s
 * @min 0
 * @max 60
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_OBC_LOSS_T, 5.0f);

/**
 * Maximum accelerometer inconsistency between IMU units that will allow arming
 *
 * @group Commander
 * @unit m/s^2
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_ARM_IMU_ACC, 0.7f);

/**
 * Maximum rate gyro inconsistency between IMU units that will allow arming
 *
 * @group Commander
 * @unit rad/s
 * @min 0.02
 * @max 0.3
 * @decimal 3
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(COM_ARM_IMU_GYR, 0.25f);

/**
 * Maximum magnetic field inconsistency between units that will allow arming
 *
 * Set -1 to disable the check.
 *
 * @group Commander
 * @unit deg
 * @min 3
 * @max 180
 */
PARAM_DEFINE_INT32(COM_ARM_MAG_ANG, 60);

/**
 * Enable mag strength preflight check
 *
 * Check if the estimator detects a strong magnetic
 * disturbance (check enabled by EKF2_MAG_CHECK)
 *
 * @value 0 Disabled
 * @value 1 Deny arming
 * @value 2 Warning only
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARM_MAG_STR, 2);

/**
 * Enable RC stick override of auto and/or offboard modes
 *
 * When RC stick override is enabled, moving the RC sticks more than COM_RC_STICK_OV
 * immediately gives control back to the pilot by switching to Position mode and
 * if position is unavailable Altitude mode.
 * Note: Only has an effect on multicopters, and VTOLs in multicopter mode.
 *
 * @min 0
 * @max 3
 * @bit 0 Enable override during auto modes (except for in critical battery reaction)
 * @bit 1 Enable override during offboard mode
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_RC_OVERRIDE, 1);

/**
 * RC stick override threshold
 *
 * If COM_RC_OVERRIDE is enabled and the joystick input is moved more than this threshold
 * the autopilot the pilot takes over control.
 *
 * @group Commander
 * @unit %
 * @min 5
 * @max 80
 * @decimal 0
 * @increment 0.05
 */
PARAM_DEFINE_FLOAT(COM_RC_STICK_OV, 30.0f);

/**
 * Require valid mission to arm
 *
 * The default allows to arm the vehicle without a valid mission.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_MIS_REQ, 0);

/**
 * Position mode navigation loss response
 *
 * This sets the flight mode that will be used if navigation accuracy is no longer adequate for position control in manual Position mode.
 *
 * @value 0 Altitude mode
 * @value 1 Land mode (descend)
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_POSCTL_NAVL, 0);

/**
 * Require arm authorization to arm
 *
 * By default off. The default allows to arm the vehicle without a arm authorization.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_REQ, 0);

/**
 * Arm authorizer system id
 *
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_ID, 10);

/**
 * Arm authorization method
 *
 * Methods:
 * - one arm: request authorization and arm when authorization is received
 * - two step arm: 1st arm command request an authorization and
 *                 2nd arm command arm the drone if authorized
 *
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 * @value 0 one arm
 * @value 1 two step arm
 */
PARAM_DEFINE_INT32(COM_ARM_AUTH_MET, 0);

/**
 * Arm authorization timeout
 *
 * Timeout for authorizer answer.
 * Used if arm authorization is requested by COM_ARM_AUTH_REQ.
 *
 * @group Commander
 * @unit s
 * @decimal 1
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_ARM_AUTH_TO, 1);

/**
 * Horizontal position error threshold.
 *
 * This is the horizontal position error (EPH) threshold that will trigger a failsafe.
 * The default is appropriate for a multicopter. Can be increased for a fixed-wing.
 * If the previous position error was below this threshold, there is an additional
 * factor of 2.5 applied (threshold for invalidation 2.5 times the one for validation).
 *
 * Set to -1 to disable.
 *
 * @unit m
 * @min -1
 * @max 400
 * @decimal 1
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_POS_FS_EPH, 5.f);

/**
 * Horizontal velocity error threshold.
 *
 * This is the horizontal velocity error (EVH) threshold that will trigger a failsafe.
 * The default is appropriate for a multicopter. Can be increased for a fixed-wing.
 * If the previous velocity error was below this threshold, there is an additional
 * factor of 2.5 applied (threshold for invalidation 2.5 times the one for validation).
 *
 * @unit m/s
 * @min 0
 * @decimal 1
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_VEL_FS_EVH, 1.f);

/**
 * Next flight UUID
 *
 * This number is incremented automatically after every flight on
 * disarming in order to remember the next flight UUID.
 * The first flight is 0.
 *
 * @group Commander
 * @category system
 * @volatile
 * @min 0
 */
PARAM_DEFINE_INT32(COM_FLIGHT_UUID, 0);

/**
 * Action after TAKEOFF has been accepted.
 *
 * The mode transition after TAKEOFF has completed successfully.
 *
 * @value 0 Hold
 * @value 1 Mission (if valid)
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_TAKEOFF_ACT, 0);

/**
 * Set GCS connection loss failsafe mode
 *
 * The GCS connection loss failsafe will only be entered after a timeout,
 * set by COM_DL_LOSS_T in seconds. Once the timeout occurs the selected
 * action will be executed.
 *
 * @value 0 Disabled
 * @value 1 Hold mode
 * @value 2 Return mode
 * @value 3 Land mode
 * @value 5 Terminate
 * @value 6 Disarm
 * @min 0
 * @max 6
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(NAV_DLL_ACT, 0);

/**
 * Set RC loss failsafe mode
 *
 * The RC loss failsafe will only be entered after a timeout,
 * set by COM_RC_LOSS_T in seconds. If RC input checks have been disabled
 * by setting the COM_RC_IN_MODE param it will not be triggered.
 *
 * @value 1 Hold mode
 * @value 2 Return mode
 * @value 3 Land mode
 * @value 5 Terminate
 * @value 6 Disarm
 * @min 1
 * @max 6
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(NAV_RCL_ACT, 2);

/**
 * RC loss exceptions
 *
 * Specify modes in which RC loss is ignored and the failsafe action not triggered.
 *
 * @min 0
 * @max 7
 * @bit 0 Mission
 * @bit 1 Hold
 * @bit 2 Offboard
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_RCL_EXCEPT, 0);

/**
 * Datalink loss exceptions
 *
 * Specify modes in which datalink loss is ignored and the failsafe action not triggered.
 *
 * @min 0
 * @max 7
 * @bit 0 Mission
 * @bit 1 Hold
 * @bit 2 Offboard
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_DLL_EXCEPT, 0);

/**
 * Set the actuator failure failsafe mode
 *
 * Note: actuator failure needs to be enabled and configured via FD_ACT_*
 * parameters.
 *
 * @min 0
 * @max 3
 * @value 0 Warning only
 * @value 1 Hold mode
 * @value 2 Land mode
 * @value 3 Return mode
 * @value 4 Terminate
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ACT_FAIL_ACT, 0);

/**
 * Expect and require a healthy MAVLink parachute system
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_PARACHUTE, 0);

/**
 * User Flight Profile
 *
 * Describes the intended use of the vehicle.
 * Can be used by ground control software or log post processing.
 * This param does not influence the behavior within the firmware. This means for example the control logic is independent of the setting of this param (but depends on other params).
 *
 * @value 0 Default
 * @value 100 Pro User
 * @value 200 Flight Tester
 * @value 300 Developer
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLT_PROFILE, 0);

/**
 * Enable checks on ESCs that report telemetry.
 *
 * If this parameter is set, the system will check ESC's online status and failures.
 * This param is specific for ESCs reporting status. It shall be used only if ESCs support telemetry.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_CHK_ESCS, 0);

/**
 * Condition to enter prearmed mode
 *
 * Condition to enter the prearmed state, an intermediate state between disarmed and armed
 * in which non-throttling actuators are active.
 *
 * @value 0 Disabled
 * @value 1 Safety button
 * @value 2 Always
 *
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_PREARM_MODE, 0);

/**
 * Enable force safety
 *
 * Force safety when the vehicle disarms
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FORCE_SAFETY, 0);

/**
 * Enable Actuator Testing
 *
 * If set, enables the actuator test interface via MAVLink (ACTUATOR_TEST), that
 * allows spinning the motors and moving the servos for testing purposes.
 *
 * @boolean
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_MOT_TEST_EN, 1);

/**
 * Timeout value for disarming when kill switch is engaged
 *
 * @group Commander
 * @unit s
 * @min 0.0
 * @max 30.0
 * @increment 0.1
 */
PARAM_DEFINE_FLOAT(COM_KILL_DISARM, 5.0f);

/**
 * Maximum allowed CPU load to still arm.
 *
 * The check fails if the CPU load is above this threshold for 2s.
 *
 * A negative value disables the check.
 *
 * @group Commander
 * @unit %
 * @min -1
 * @max 100
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_CPU_MAX, 95.0f);

/**
 * Maximum allowed RAM usage to pass checks
 *
 * The check fails if the RAM usage is above this threshold.
 *
 * A negative value disables the check.
 *
 * @group Commander
 * @unit %
 * @min -1
 * @max 100
 * @increment 1
 */
PARAM_DEFINE_FLOAT(COM_RAM_MAX, 95.0f);

/**
 * Required number of redundant power modules
 *
 * This configures a check to verify the expected number of 5V rail power supplies are present. By default only one is expected.
 * Note: CBRK_SUPPLY_CHK disables all power checks including this one.
 *
 * @group Commander
 * @min 0
 * @max 4
 */
PARAM_DEFINE_INT32(COM_POWER_COUNT, 1);

/**
 * Timeout for detecting a failure after takeoff
 *
 * A non-zero, positive value specifies the timeframe in seconds within failure detector is allowed to disarm the vehicle
 * if attitude exceeds the limits defined in FD_FAIL_P and FD_FAIL_R.
 * The check is not executed for flight modes that do support acrobatic maneuvers, e.g: Acro (MC/FW) and Manual (FW).
 * A zero or negative value means that the check is disabled.
 *
 * @group Commander
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(COM_LKDOWN_TKO, 3.0f);

/**
 * Enable FMU SD card detection check
 *
 * This check detects if the FMU SD card is missing.
 * Depending on the value of the parameter, the check can be
 * disabled, warn only or deny arming.
 *
 * @group Commander
 * @value 0 Disabled
 * @value 1 Warning only
 * @value 2 Enforce SD card presence
 */
PARAM_DEFINE_INT32(COM_ARM_SDCARD, 1);

/**
 * Enable FMU SD card hardfault detection check
 *
 * This check detects if there are hardfault files present on the
 * SD card. If so, and the parameter is enabled, arming is prevented.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_ARM_HFLT_CHK, 1);

/**
 * Enable Drone ID system detection and health check
 *
 * This check detects if the Open Drone ID system is missing.
 * Depending on the value of the parameter, the check can be
 * disabled, warn only or deny arming.
 *
 * @group Commander
 * @value 0 Disabled
 * @value 1 Warning only
 * @value 2 Enforce Open Drone ID system presence
 */
PARAM_DEFINE_INT32(COM_ARM_ODID, 0);

/**
 * Enforced delay between arming and further navigation
 *
 * The minimal time from arming the motors until moving the vehicle is possible is COM_SPOOLUP_TIME seconds.
 * Goal:
 * - Motors and propellers spool up to idle speed before getting commanded to spin faster
 * - Timeout for ESCs and smart batteries to successfulyy do failure checks
 *   e.g. for stuck rotors before the vehicle is off the ground
 *
 * @group Commander
 * @min 0
 * @max 30
 * @decimal 1
 * @increment 0.1
 * @unit s
 */
PARAM_DEFINE_FLOAT(COM_SPOOLUP_TIME, 1.0f);

/**
 * Wind speed warning threshold
 *
 * A warning is triggered if the currently estimated wind speed is above this value.
 * Warning is sent periodically (every 1 minute).
 *
 * Set to -1 to disable.
 *
 * @min -1
 * @decimal 1
 * @increment 0.1
 * @group Commander
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(COM_WIND_WARN, -1.f);

/**
 * Maximum allowed flight time
 *
 * The vehicle aborts the current operation and returns to launch when
 * the time since takeoff is above this value. It is not possible to resume the
 * mission or switch to any auto mode other than RTL or Land. Taking over in any manual
 * mode is still possible.
 *
 * Starting from 90% of the maximum flight time, a warning message will be sent
 * every 1 minute with the remaining time until automatic RTL.
 *
 * Set to -1 to disable.
 *
 * @unit s
 * @min -1
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_FLT_TIME_MAX, -1);

/**
 * High wind speed failsafe threshold
 *
 * Wind speed threshold above which an automatic failsafe action is triggered.
 * Failsafe action can be specified with COM_WIND_MAX_ACT.
 *
 * @min -1
 * @decimal 1
 * @increment 0.1
 * @group Commander
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(COM_WIND_MAX, -1.f);

/**
 * High wind failsafe mode
 *
 * Action the system takes when a wind speed above the specified threshold is detected.
 * See COM_WIND_MAX to set the failsafe threshold.
 * If enabled, it is not possible to resume the mission or switch to any auto mode other than
 * RTL or Land if this threshold is exceeded. Taking over in any manual
 * mode is still possible.
 *
 * @group Commander
 *
 * @value 0 None
 * @value 1 Warning
 * @value 2 Hold
 * @value 3 Return
 * @value 4 Terminate
 * @value 5 Land
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_WIND_MAX_ACT, 0);

/**
 * Low position accuracy failsafe threshold
 *
 * This triggers the action specified in COM_POS_LOW_ACT if the estimated position accuracy is below this threshold.
 * Local position has to be still declared valid, which requires some kind of velocity aiding or large dead-reckoning time (EKF2_NOAID_TOUT),
 * and a high failsafe threshold (COM_POS_FS_EPH).
 *
 * Set to -1 to disable.
 *
 * @min -1
 * @max 1000
 * @group Commander
 * @unit m
 */
PARAM_DEFINE_FLOAT(COM_POS_LOW_EPH, -1.0f);

/**
 * Low position accuracy action
 *
 * Action the system takes when the estimated position has an accuracy below the specified threshold.
 * See COM_POS_LOW_EPH to set the failsafe threshold.
 * The failsafe action is only executed if the vehicle is in auto mission or auto loiter mode,
 * otherwise it is only a warning.
 *
 * @group Commander
 *
 * @value 0 None
 * @value 1 Warning
 * @value 2 Hold
 * @value 3 Return
 * @value 4 Terminate
 * @value 5 Land
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_POS_LOW_ACT, 3);

/**
 * Flag to allow arming
 *
 * Set 0 to prevent accidental use of the vehicle e.g. for safety or maintenance reasons.
 *
 * @boolean
 * @value 0 Disallow arming
 * @value 1 Allow arming
 * @group Commander
 */
PARAM_DEFINE_INT32(COM_ARMABLE, 1);

/**
 * Minimum battery level for arming
 *
 * Threshold for battery percentage below arming is prohibited.
 *
 * A negative value means BAT_CRIT_THR is the threshold.
 *
 * @unit norm
 * @min -1
 * @max 0.9
 * @decimal 2
 * @increment 0.01
 * @group Commander
 */
PARAM_DEFINE_FLOAT(COM_ARM_BAT_MIN, -1.f);

/**
 * Enable throw-start
 *
 * Allows to start the vehicle by throwing it into the air.
 *
 * @group Commander
 * @boolean
 */
PARAM_DEFINE_INT32(COM_THROW_EN, 0);

/**
 * Minimum speed for the throw start
 *
 * When the throw launch is enabled, the drone will only allow motors to spin after this speed
 * is exceeded before detecting the freefall. This is a safety feature to ensure the drone does
 * not turn on after accidental drop or a rapid movement before the throw.
 *
 * Set to 0 to disable.
 *
 * @group Commander
 * @min 0
 * @decimal 1
 * @increment 0.1
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(COM_THROW_SPEED, 5);

/**
 * Remaining flight time low failsafe
 *
 * Action the system takes when the remaining flight time is below
 * the estimated time it takes to reach the RTL destination.
 *
 * @group Commander
 * @value 0 None
 * @value 1 Warning
 * @value 3 Return
 * @increment 1
 */
PARAM_DEFINE_INT32(COM_FLTT_LOW_ACT, 3);

/**
 * Allow external mode registration while armed.
 *
 * By default disabled for safety reasons
 *
 * @group Commander
 * @boolean
 *
 */
PARAM_DEFINE_INT32(COM_MODE_ARM_CHK, 0);

/**
 * 辨识模式
 *
 * 选择当前固件运行哪种辨识流程。1/2 为单电机推力阶跃辨识，3/4 为自适应舵机扫频辨识。
 * 设为 3 时 control_allocator 自动启动 fs_uart_servo 总线舵机驱动，其它模式自动关闭。
 *
 * @group 电机/舵机辨识
 * @value 0 关闭
 * @value 1 电机辨识：定时升档（每档保持 IDEN_STEP_TIME 秒）
 * @value 2 电机辨识：AUX2 上升沿升档（由 <=0 变为 >0 升一档，下降沿保持）
 * @value 3 总线舵机扫频（FashionStar UART，AUX1 门控，目标 = IDEN_SV_IDX）
 * @value 4 PWM 舵机扫频（AUX1 门控，目标 = IDEN_SV_IDX）
 */
PARAM_DEFINE_INT32(IDEN_TYPE, 0);

/**
 * 辨识目标电机序号
 *
 * 电机辨识（IDEN_TYPE=1/2）时被激励的电机序号，1 基，对应控制分配中的电机顺序。
 * 其余电机输出保持为 0。
 *
 * @group 电机/舵机辨识
 * @min 1
 * @max 12
 */
PARAM_DEFINE_INT32(IDEN_MOTOR_IDX, 1);

/**
 * 电机辨识每档保持时间
 *
 * IDEN_TYPE=1 定时升档模式下，每一个推力档位的保持时长。
 * 推力指令从 0.1 到 1.0，步长 0.05，共 19 档。
 *
 * @group 电机/舵机辨识
 * @unit s
 * @min 0.1
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IDEN_STEP_TIME, 5.0f);

/**
 * 舵机辨识 AUX1 门限
 *
 * 舵机辨识（IDEN_TYPE=3/4）时，遥控 AUX1（manual_control_setpoint.aux1，范围 -1..1）
 * 高于该值才允许进入自适应扫频。电机辨识（IDEN_TYPE=1/2）固定使用 aux1 > 0 判断，不受此参数影响。
 *
 * @group 电机/舵机辨识
 * @min -1.0
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IDEN_AUX_THR, 0.3f);

/**
 * 辨识目标舵机通道
 *
 * 舵机辨识时被激励的 actuator_servos.control[] 通道下标（0–7）。
 * IDEN_TYPE=3：为 FashionStar 总线舵机 ID（与 fs_uart_servo 协议中的 servo_id 一致）；
 * IDEN_TYPE=4：为 PWM Servo 通道（Servo1…Servo8，需与 PWM_MAIN_FUNCx / PWM_AUX_FUNCx 映射一致）。
 * 例如设为 1 仅激励 1 号通道，设为 2 仅激励 2 号通道。
 *
 * @group 电机/舵机辨识
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(IDEN_SV_IDX, 1);

/**
 * 总线舵机辨识控制频率
 *
 * 仅 IDEN_TYPE=3：扫频期间同步角指令的发送频率。回传自动按同一频率查询目标舵机（一发一问）。
 * 与 FS_UT_R_HZ 无关；平时跟控仍由 FS_UT_R_HZ / 200 Hz 上限决定。
 * PWM 舵机辨识（IDEN_TYPE=4）不使用本参数。
 * 建议不低于 6×IDEN_CH_F1（扫到 30 Hz 时用 200 Hz）；过低则高频段欠采样。
 *
 * @group 电机/舵机辨识
 * @unit Hz
 * @min 20
 * @max 200
 * @increment 10
 */
PARAM_DEFINE_INT32(IDEN_SV_HZ, 200);

/**
 * 舵机扫频起始频率
 *
 * 对数扫频与引导正弦的起始频率。位置伺服再往下增益恒为 1，没有额外信息。
 *
 * @group 电机/舵机辨识
 * @unit Hz
 * @min 0.05
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IDEN_CH_F0, 0.2f);

/**
 * 舵机扫频上限频率
 *
 * 实际扫到 min(本参数, 由速率自测得到的 f1_eff)。无反馈时即用本值。
 *
 * @group 电机/舵机辨识
 * @unit Hz
 * @min 1.0
 * @max 80.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(IDEN_CH_F1, 30.0f);

/**
 * 单次扫频时长
 *
 * 每一次全幅或半幅对数扫频的时长（不含引导与段间停顿）。
 *
 * @group 电机/舵机辨识
 * @unit s
 * @min 10.0
 * @max 600.0
 * @decimal 0
 */
PARAM_DEFINE_FLOAT(IDEN_CH_DUR, 90.0f);

/**
 * 舵机扫频低频幅值
 *
 * 低于速率饱和边界时使用的固定幅值（度）。高频段按 0.6·Vmax/(2πf) 自动缩小，且不低于 IDEN_CH_AMIN。
 *
 * @group 电机/舵机辨识
 * @unit deg
 * @min 1.0
 * @max 45.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(IDEN_CH_AMP, 8.0f);

/**
 * 舵机扫频最小幅值
 *
 * 幅值触底时对应的频率即为有效扫频上限 f1_eff。保证回传量化噪声下仍有足够信噪比。
 *
 * @group 电机/舵机辨识
 * @unit deg
 * @min 0.2
 * @max 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(IDEN_CH_AMIN, 1.5f);

/**
 * 速率自测阶跃幅值
 *
 * 静置后按 0 → +S → −S → 0 各保持 1 秒，用陀螺或总线回传估计峰值角速度 Vmax。
 *
 * @group 电机/舵机辨识
 * @unit deg
 * @min 2.0
 * @max 45.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(IDEN_CH_STEP, 15.0f);

/**
 * 全幅扫频重复次数
 *
 * 用于计算相干函数。总时长约 1+4+N×(2/F0+DUR+2) 秒，另加半幅扫频（若开启）。
 *
 * @group 电机/舵机辨识
 * @min 1
 * @max 5
 */
PARAM_DEFINE_INT32(IDEN_CH_REP, 3);

/**
 * 半幅扫频（线性性检验）
 *
 * 全幅重复完成后，再用一半幅值扫一次。两组频响重合则线性假设成立。
 *
 * @group 电机/舵机辨识
 * @boolean
 */
PARAM_DEFINE_INT32(IDEN_CH_HALF, 1);
