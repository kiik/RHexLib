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
 * $Id: SimMotor.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Low level hardware interface to the DC motors in the SimSect interface
 *
 * Created       : Uluc Saranli, 03/22/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifdef _LINUX_      // Only compile this file for Linux

#include <math.h>
#include "ModuleManager.hh"
#include "SimSectHW.hh"
#include "SimComponents.hh"
#include "SimSectDriver.hh"

//
// SimMotor::SimMotor : Class constructor
//
SimMotor::SimMotor( void ) {

  MMMessage( "Initializing DC motor interfaces..." );

  // Determine the mode of operation
  controlMode = VOLTAGE_MODE;

  MMMessage( "done.\n" );

}

// SimMotor::getParams : Returns hardware specific constant parameters
void SimMotor::getParams( uint index, MotorParam_t *params ) {

  params->torqueConstant = SIMSECT_TORQUE_CONST;
  params->speedConstant = SIMSECT_SPEED_CONST;
  params->terminalResistance = SIMSECT_RA;

  params->encRatio = SIMSECT_ENC_RATIO;
  params->gearRatio = SIMSECT_GEAR_RATIO;

}

float SimMotor::getVoltage( uint index ) {

  // Read the motor terminal voltage from the SimSectDriver class
  return driver->motorVoltage( index );

};

float SimMotor::getCurrent( uint index ) {

  // Read the motor armature current from the SimSectDriver class
  return driver->motorCurrent( index ); 

};

float SimMotor::getBackEMF( uint index ) {

  // Read the motor back EMF from the SimSectDriver class
  return driver->motorBackEMF( index ); 

};

//
// Sets the command to the motor based on the control mode and the
// calibration parameters.
//
void SimMotor::setCommand( uint index, float cmd ) {

  // Set the command through the SimSect interface class.
  // Note that the control mode will be determined by SimSectDriver and the
  // command will be appropriately clipped.
  driver->setCommand( index, cmd );

};

float SimMotor::getCommand( uint index ) {

  // Get the latest command from SimSectDriver.
  return driver->getCommand( index );
}

#endif
