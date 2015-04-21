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
 * $Id: StdModules.cc,v 1.5 2001/08/06 20:22:58 ulucs Exp $
 *
 * Standard RHex moduleset
 *
 * This file contains utility functions to create and add all the standard 
 * modules in RHexLib with their default orders, periods and offsets.
 *
 * Created       : Uluc Saranli, 03/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "sysutil.hh"
#include "StdModules.hh"
#include "ModuleManager.hh"
#include "EncoderReader.hh"
#include "SpeedFilter.hh"
#include "AnalogOutput.hh"
#include "PositionControl.hh"
#include "StallSensor.hh"
#include "SlopeEstimator.hh"
#include "RemoteControl.hh"

#include "CalibMachine.hh"

#include "StandMachine.hh"
#include "SitMachine.hh"
#include "WalkMachine.hh"
#include "RHexWalker.hh"
#include "StartupMachine.hh"

#include "RHexLogger.hh"

// Global module pointers for easy access

EncoderReader     * encreader[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
AnalogOutput      * analogout[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
SpeedFilter       * spdfilter[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
PositionControl   * poscontrol[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
StallSensor       * stallsensor[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
RemoteControl     * rhexrc = NULL;
SlopeEstimator    * slopeestimator = NULL;

CalibMachine      * calibmachine[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
StandMachine      * standmachine = NULL;
SitMachine        * sitmachine = NULL;
WalkMachine       * walkmachine = NULL;
RHexWalker        * rhexwalker = NULL;
StartupMachine    * startupmachine = NULL;

RHexLogger        * rhexlogger = NULL;

void RHexAddStdModules( void ) {

  int i;

  // set up basic modules
  for ( i = 0; i < 6; i++ ) {
    encreader[i] = new EncoderReader ( i );
    analogout[i] = new AnalogOutput ( i );
    spdfilter[i] = new SpeedFilter ( i );
    poscontrol[i] = new PositionControl ( i );
    stallsensor[i] = new StallSensor ( i );
    calibmachine[i] = new CalibMachine ( i );
  }

  rhexrc = new RemoteControl ( REMOTECONTROL_NAME );
  slopeestimator = new SlopeEstimator;

  standmachine = new StandMachine;
  sitmachine = new SitMachine;
  walkmachine = new WalkMachine;
  rhexwalker = new RHexWalker;
  startupmachine = new StartupMachine;

  rhexlogger = new RHexLogger;

  // add modules ( module, period, offset, order );
  for ( i = 0; i < 6; i++ ) {
    MMAddModule ( analogout[i], ANALOGOUTPUT_PERIOD, 
                  ANALOGOUTPUT_OFFSET, ANALOGOUTPUT_ORDER );
    MMAddModule ( encreader[i], ENCODERREADER_PERIOD, 
                  ENCODERREADER_OFFSET, ENCODERREADER_ORDER );
    MMAddModule ( spdfilter[i], SPEEDFILTER_PERIOD, 
                  SPEEDFILTER_OFFSET, SPEEDFILTER_ORDER );
    MMAddModule ( poscontrol[i], POSITIONCONTROL_PERIOD, 
                  POSITIONCONTROL_OFFSET, POSITIONCONTROL_ORDER  );
    MMAddModule ( stallsensor[i], STALLSENSOR_PERIOD, 
                  STALLSENSOR_OFFSET, STALLSENSOR_ORDER );
    MMAddModule ( calibmachine[i], CALIBMACHINE_PERIOD, 
                  CALIBMACHINE_OFFSET, CALIBMACHINE_ORDER );
  }

  MMAddModule ( rhexrc, REMOTECONTROL_PERIOD, 
                ENCODERREADER_OFFSET, REMOTECONTROL_ORDER );
  MMAddModule ( slopeestimator, SLOPEESTIMATOR_PERIOD, 
                SLOPEESTIMATOR_OFFSET, SLOPEESTIMATOR_ORDER );

  MMAddModule ( sitmachine, SITMACHINE_PERIOD, 
                SITMACHINE_OFFSET, SITMACHINE_ORDER );
  MMAddModule ( standmachine, STANDMACHINE_PERIOD, 
                STANDMACHINE_OFFSET, STANDMACHINE_ORDER );
  MMAddModule ( walkmachine, WALKMACHINE_PERIOD, 
                WALKMACHINE_OFFSET, WALKMACHINE_ORDER );
  MMAddModule ( rhexwalker, RHEXWALKER_PERIOD, 
                RHEXWALKER_OFFSET, RHEXWALKER_ORDER );
  MMAddModule ( startupmachine, STARTUPMACHINE_PERIOD, 
                STARTUPMACHINE_OFFSET, STARTUPMACHINE_ORDER );

  MMAddModule ( rhexlogger, RHEXLOGGER_PERIOD, 
                RHEXLOGGER_OFFSET, RHEXLOGGER_ORDER );

}

void RHexRemoveStdModules( void ) {

  int i;

  // Remove the modules
  for ( i = 0; i < 6; i++ ) {
    MMRemoveModule( analogout[i] );
    MMRemoveModule( encreader[i] );
    MMRemoveModule( spdfilter[i] );
    MMRemoveModule( poscontrol[i]  );
    MMRemoveModule( stallsensor[i] );
    MMRemoveModule( calibmachine[i] );
  }

  MMRemoveModule( rhexrc );
  MMRemoveModule( slopeestimator );

  MMRemoveModule( sitmachine );
  MMRemoveModule( standmachine );
  MMRemoveModule( walkmachine );
  MMRemoveModule( rhexwalker );
  MMRemoveModule( startupmachine );

  MMRemoveModule( rhexlogger );

  // deallocate module objects
  for ( i = 0; i < 6; i++ ) {
    if ( encreader[i] ) delete encreader[i];
    if ( analogout[i] ) delete analogout[i];
    if ( spdfilter[i] ) delete spdfilter[i];
    if ( poscontrol[i] ) delete poscontrol[i];
    if ( stallsensor[i] ) delete stallsensor[i];
    if ( calibmachine[i] ) delete calibmachine[i];
  }

  if ( rhexrc ) delete rhexrc;
  if ( slopeestimator ) delete slopeestimator;

  if ( standmachine ) delete standmachine;
  if ( sitmachine ) delete sitmachine;
  if ( walkmachine ) delete walkmachine;
  if ( rhexwalker ) delete rhexwalker;
  if ( startupmachine ) delete startupmachine;

  if ( rhexlogger ) delete rhexlogger;

}

