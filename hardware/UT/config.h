/*
 * This file is part of RHexLib, 
 *
 * Copyright (c) 2001 The University of Michigan, its Regents,
 * Fellows, Employees and Agents. All rights reserved, and distributed as
 * free software under the following license.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1) Redistributions of source code must retain the above copyright
 * notice, this list of conditions, the following disclaimer and the
 * file called "CREDITS" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called "CREDITS" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** @file config.h
 *  @brief Low level hardware configuration header for the University of Tartu version of RHex
 *
 *  @author Meelik Kiik
 *  @version 0.1
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#ifdef _QNX4_
#pragma off(unreferenced)
#endif

// ---------------------------------------------------------------
// Digital IO assignments ----------------------------------------

// Drive enable digital lines
#define DRIVE_ENABLE  16
#define DRIVE_START   17

// Home switches
const int HallSwitchBytes[6] = { 7, 8, 8, 7, 6, 6 };
const int HallSwitchBits[6] = { 3, 2, 3, 2, 3, 2 };

// Pulse width gate signal lines for the RC inputs
#define RC_GATE0     46
#define RC_GATE1     47
#define RC_GATE2     45
#define RC_GATE3     38
#define RC_GATE4     44
#define RC_GATE5     39

// Pulse width gate signal lines for the accelerometers
#define ACCEL_GATEX  71
#define ACCEL_GATEY  69
#define ACCEL_GATEZ  70

// ---------------------------------------------------------------
// Analog IO assignments -----------------------------------------

// Analog channels for the battery voltage and current
#define BATT_V_CHANNEL 1
#define BATT_I_CHANNEL 0

// ---------------------------------------------------------------
// Timer allocations ---------------------------------------------

// Pulse width measurement timers for the RC inputs
#define RC_TIMER0     4
#define RC_TIMER1     3
#define RC_TIMER2     5
#define RC_TIMER3     2
#define RC_TIMER4     0
#define RC_TIMER5     1

// Pulse width measurement timers for the accelerometers
#define ACCEL_TIMERX  6
#define ACCEL_TIMERY  8
#define ACCEL_TIMERZ  7

// ---------------------------------------------------------------
// RC timing info ------------------------------------------------
// Minimum and maximum pulse widths for the RC signals (in s)
// and the midpoint and range of the RC PWM signals. 
#define RC_MIN_WIDTH 0.000500
#define RC_MAX_WIDTH 0.002500
#define RC_MID_WIDTH 0.001516
#define RC_RANGE 0.001

// Timeout for the RC PWM. Maximum length of time the gate signal can
// remain low before the input PWM is ignored. (in microseconds )
#define RC_TIMEOUT 0.5

// ---------------------------------------------------------------
// Accelerometer timing info -------------------------------------
// Timeout for the Accelerometer PWM. Maximum length of time the gate
// signal can remain low before the input PWM is ignored. (in
// microseconds )
#define ACCEL_TIMEOUT 0.5

// ---------------------------------------------------------------
// Internal module config ----------------------------------------

// Pulse Width measurement for the RC and accelerometer signals
#define PWM_MODULE_PERIOD 1
#define PWM_MODULE_ORDER 0
#define PWM_MODULE_OFFSET 0

// Analog probing
#define ADC_MODULE_PERIOD 1
#define ADC_MODULE_ORDER  0
#define ADC_MODULE_OFFSET 0

// Default ADC polling mode and number of channels
#define ADC_POLLING       true
#define ADC_CHANNELS      5

// ---------------------------------------------------------------
// PC104 stack configuration -------------------------------------

// First encoder card (ENCI) configuration
#define ENCI_IOBASE  (0x320)
#define ENCI_IRQ     10

// Second encoder card (ENCII) configuration
#define ENCII_IOBASE  (0x300)
#define ENCII_IRQ     10

// Analog/digital I/O card (ADIO) configuration
#define ADIO_IOBASE    (0x3a0)
#define ADIO_TIMER_IRQ 5
#define ADIO_ADC_IRQ   7

// Watchdog timer stuff

#define WATCHDOG_INT        0x52 // DOS software INT
#define WATCHDOG_ENABLE     0x0c // Enable command
#define WATCHDOG_DISABLE    0x0d // Disable command
#define WATCHDOG_RESET      0x0e // Reset command

// ---------------------------------------------------------------
// Analog IO configuration parameters ----------------------------

// Analog Output limits (V)
#define AOUT_MIN     0.0
#define AOUT_MAX     10.0

#define DAC_BITS     12 // DAC bit precision 

// ---------------------------------------------------------------
// Motor physical parameters -------------------------------------


#define MAX_SPEED    1047.2    // No load motor speed (in rad/s)
#define MAX_TORQUE   0.218     // Stall torque (in Nm)
#define RA           1.33      // Armature resistance (in ohm )
#define TORQUE_CONST 0.0161    // Torque constant ( Nm / A )
#define SPEED_CONST  55.3685   // Speed constant ( (rad/s) / V )
#define TIME_CONST   5.0       // Mechanical time constant (in ms)
#define INERTIA      9.11e-7   // Rotor inertia (kg m^2)
#define LA           0.12      // Armature inductance (mH)
#define GEAR_RATIO   (1.0 / 33.0625)// Gear ratio (output rev / shaft rev)
#define ENC_RATIO    2000      // Encoder ratio (count/rev)

// ---------------------------------------------------------------
// Motor drive parameters ----------------------------------------

#define DRIVE_DROP   0.75        // Motor drive voltage drop coefficient (V/A)
#define DRIVE_SCALE  (2.0 / 4.0) // Motor drive PWM output  scaling factor ([-1,1]/V)
#define DRIVE_SUPPLY 24.0        // Motor drive voltage suppy value (V)
#define DRIVE_OFFSET 5.7         // Motor drive  voltage offset (V)


#endif // #ifndef _CONFIG_HH
