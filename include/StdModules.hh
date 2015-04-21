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

/*********************************************************************
 * $Id: StdModules.hh,v 1.5 2001/08/06 20:22:32 ulucs Exp $
 *
 * Standard RHex moduleset
 *
 * This file contains the definition for the set of stahndard RHex 
 * modules. It defines their orders, periods, offsets and names.
 *
 * Created       : Uluc Saranli, 03/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _STDMODULES_HH
#define _STDMODULES_HH


// The standard ranges for the module orders are as follows:
//
// 0     -  9999 : Sensing modules
// 10000 - 19999 : User defined controller modules
// 20000 - 24999 : RHexLib behavioral controller modules 
// 25000 - 29999 : RHexLib low level controller modules 
// 30000 - 39999 : Actuator modules ( analog outputs etc. )
// 40000 - 44999 : Logging modules
// 45000 -       : other modules

#define SENSING_MODULES         0
#define USER_DEFINED_SENSORS    5000
#define USER_CONTROLLERS        10000
#define BEHAVIORAL_CONTROLLERS  20000
#define LOW_LEVEL_CONTROLLERS   25000
#define ACTUATOR_MODULES        30000
#define LOGGING_MODULES         40000
#define OTHER_MODULES           45000

// ------------------------------------------------------------------
// Sensing modules ------------------------------------------------

// The PulseWidth module --------------------------------
#define PULSEWIDTH_NAME    "pulsewidth"
#define PULSEWIDTH_ORDER   0
#define PULSEWIDTH_PERIOD  1
#define PULSEWIDTH_OFFSET  0

// The RemoteControl modules ----------------------------
#define REMOTECONTROL_NAME      "rhexrc"
#define REMOTECONTROL_ORDER     500
#define REMOTECONTROL_PERIOD    1
#define REMOTECONTROL_OFFSET    0

// The EncoderReader module -----------------------------
#define ENCODERREADER_NAME    "encreader"
#define ENCODERREADER_ORDER   1000
#define ENCODERREADER_PERIOD  1
#define ENCODERREADER_OFFSET  0

// The SpeedFilter module -------------------------------
#define SPEEDFILTER_NAME      "spdfilter"
#define SPEEDFILTER_ORDER     1500
#define SPEEDFILTER_PERIOD    1
#define SPEEDFILTER_OFFSET    0

// The StallSensor module -------------------------------
#define STALLSENSOR_NAME      "stallsensor"
#define STALLSENSOR_ORDER     2500
#define STALLSENSOR_PERIOD    1
#define STALLSENSOR_OFFSET    0

// The SlopeEstimator module ----------------------------
#define SLOPEESTIMATOR_NAME     "slopeestimator"
#define SLOPEESTIMATOR_ORDER     2500
#define SLOPEESTIMATOR_PERIOD    1
#define SLOPEESTIMATOR_OFFSET    0

// The Filter modules -----------------------------------
#define FILTERMODULE_ORDER     3000
#define FILTERMODULE_PERIOD    1
#define FILTERMODULE_OFFSET    0

// ------------------------------------------------------------------
// User defined sensors ---------------------------------------------

// ------------------------------------------------------------------
// User defined controllers -----------------------------------------

// ------------------------------------------------------------------
// RHexLib behavioral controller modules  ---------------------------

// The StartupMachine module ----------------------------

#define STARTUPMACHINE_NAME     "startupmachine"
#define STARTUPMACHINE_ORDER    20000
#define STARTUPMACHINE_PERIOD   1
#define STARTUPMACHINE_OFFSET   0

// The CalibMachine module ------------------------------

#define CALIBMACHINE_NAME       "calibmachine"
#define CALIBMACHINE_ORDER      21000
#define CALIBMACHINE_PERIOD     1
#define CALIBMACHINE_OFFSET     0

// The SitMachine module -------------------------------

#define SITMACHINE_NAME         "sitmachine"
#define SITMACHINE_ORDER        22000
#define SITMACHINE_PERIOD       1
#define SITMACHINE_OFFSET       0

// The StandMachine module ------------------------------

#define STANDMACHINE_NAME       "standmachine"
#define STANDMACHINE_ORDER      22000
#define STANDMACHINE_PERIOD     1
#define STANDMACHINE_OFFSET     0

// The WalkMachine module -------------------------------

#define WALKMACHINE_NAME        "walkmachine"
#define WALKMACHINE_ORDER       22000
#define WALKMACHINE_PERIOD      1
#define WALKMACHINE_OFFSET      0

// The RHexWalker module -------------------------------

#define RHEXWALKER_NAME        "rhexwalker"
#define RHEXWALKER_ORDER       22000
#define RHEXWALKER_PERIOD      1
#define RHEXWALKER_OFFSET      0

// ------------------------------------------------------------------
// RHexLib low level controller modules  ---------------------------

// The MotorControl modules -----------------------------

#define MOTORCONTROL_ORDER    25000
#define MOTORCONTROL_PERIOD   1
#define MOTORCONTROL_OFFSET   0

// The PositionControl module ---------------------------

#define POSITIONCONTROL_NAME    "poscontrol"
#define POSITIONCONTROL_ORDER   25000
#define POSITIONCONTROL_PERIOD  1
#define POSITIONCONTROL_OFFSET  0

// ------------------------------------------------------------------
// Actuator modules ( analog outputs etc. ) -------------------------

// The AnalogOutput module ------------------------------

#define ANALOGOUTPUT_NAME     "analogout"
#define ANALOGOUTPUT_ORDER    30000
#define ANALOGOUTPUT_PERIOD   1
#define ANALOGOUTPUT_OFFSET   0

// ------------------------------------------------------------------
// Logging modules  -------------------------------------------------

// The DataLogger modules ------------------------------

#define DATALOGGER_NAME       "datalogger"
#define DATALOGGER_ORDER      40000
#define DATALOGGER_PERIOD     1
#define DATALOGGER_OFFSET     0

#define RHEXLOGGER_NAME       "rhexlogger"
#define RHEXLOGGER_ORDER      40000
#define RHEXLOGGER_PERIOD     1
#define RHEXLOGGER_OFFSET     0


void RHexAddStdModules( void );
void RHexRemoveStdModules( void );

// Global variables for all the added standard modules
extern class EncoderReader     * encreader[6];
extern class AnalogOutput      * analogout[6];
extern class SpeedFilter       * spdfilter[6];
extern class PositionControl   * poscontrol[6];
extern class StallSensor       * stallsensor[6];
extern class SlopeEstimator    * slopeestimator;

extern class CalibMachine      * calibmachine[6];
extern class StandMachine      * standmachine;
extern class SitMachine        * sitmachine;
extern class WalkMachine       * walkmachine;
extern class RHexWalker        * rhexwalker;
extern class StartupMachine    * startupmachine;

extern class RHexLogger        * rhexlogger;

#endif
