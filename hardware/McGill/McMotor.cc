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
 * $Id: McMotor.cc,v 1.6 2001/07/26 06:52:22 ulucs Exp $
 *
 * Low level hardware interface to the DC motors in the Michigan hardware
 *
 * Created       : Uluc Saranli, 03/22/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <math.h>
#include "ModuleManager.hh"
#include "basicmath.hh"
#include "StdModules.hh"
#include "config.hh"
#include "McGillHW.hh"
#include "McComponents.hh"

//
// McMotor::McMotor : Class constructor
//
McMotor::McMotor( Hardware *hw ) {

  int i;

  MMMessage( "Initializing DC motor interfaces...\n" );

  // Determine whether motor states will be estimated or not. Disabled
  // by default.
  stateEstimation = bool( MMGetFloatSymbol( "motor_estimation_enable", 0.0 ) );
  if ( ! stateEstimation )
    MMMessage( "McMotor::McMotor : Motor state estimation disabled!\n" );

  // Determine whether the voltage drop on the motor drives will be
  // compensated for.
  driveCompensation 
    = bool( MMGetFloatSymbol( "drive_compensation_enable", 1.0 ) );
  if ( ! driveCompensation )
    MMMessage( "MMotor::MMotor : Motor drive drop compensation disabled!\n" );

  // Retrieve necessary hardware components
  analogIO = (McAnalog * ) hw->analogIO;

  // Choose the mode of operation
  for ( i = 0; i < 6; i++ )
    controlMode[i] = VOLTAGE_MODE;

  // Initialize internal data
  for ( i = 0; i < 6; i++ ) {
    voltage[i] = current[i] = backEMF[i] = temperature[i] = 0.0;
    lastSpeed[i] = lastOutput[i] = 0.0;
    encoder[i] = NULL;
  }
  backEMFcoeff = 1.0 / (SPEED_CONST * GEAR_RATIO );

  // Initialize the default calibration information
  for ( i = 0; i < 6; i++ ) {

    calib[ i ].driveOffset = DRIVE_OFFSET;

    // Note that the polorities are determined by how MEncoder::read()
    // reverses polarity of one side!
    if ( i < 3 )
      calib[ i ].drivePolarity = +1;
    else
      calib[ i ].drivePolarity = -1;
  }

}


// McMotor::getParams : Returns hardware specific constant parameters
void McMotor::getParams( uint index, MotorParam_t *params ) {

  params->torqueConstant = TORQUE_CONST;
  params->speedConstant = SPEED_CONST;
  params->terminalResistance = RA;

  params->encRatio = ENC_RATIO;
  params->gearRatio = GEAR_RATIO;

}
float McMotor::getVoltage( uint index ) { 

  // If estimation is enabled, compute the motor states
  if ( stateEstimation )
    computeStates( index );
  else
    MMWarning( "MMotor::getVoltage", "Motor state estimation disabled!" );

  return voltage[index]; 

};

float McMotor::getCurrent( uint index ) {

  // If estimation is enabled, compute the motor states
  if ( stateEstimation )
    computeStates( index );
  else
    MMWarning( "MMotor::getCurrent", "Motor state estimation disabled!" );

  return current[index]; 

};

float McMotor::getBackEMF( uint index ) {

  // If estimation is enabled, compute the motor states
  if ( stateEstimation )
    computeStates( index );
  else
    MMWarning( "McMotor::getBackEMF", "Motor state estimation disabled!" );

  return backEMF[index]; 

};

//
// Sets the command to the motor based on the control mode and the
// calibration parameters.
//
void McMotor::setCommand( uint index, float cmd ) {

  float apexCmd;
  float duty, backEMF;
  float adjSupply;

  // Clip the command to an appropriate range and record the result
  command[index] = fmax( fmin( DRIVE_SUPPLY, cmd ), - DRIVE_SUPPLY );

  backEMF = getBackEMF( index );

  if ( driveCompensation ) {
    // Motor drive voltage drop compensation is enabled. Do fancy things.

    // The following code segment figures out what command voltage needs
    // to be output to the APEX drives in order to obtain the desired
    // terminal voltage on the motor. It incorporates a model for the
    // voltage drop on the APEX chip.
    //
    // duty = { Vo / (Vs - Rd * (Vo - Vemf) / Ra )   if Vo > 0
    //        {
    //        { Vo / (Vs + Rd * (Vo - Vemf) / Ra )   if Vo < 0
    //
    // Where Vo is the desired terminal voltage, Vs is the APEX supply voltage
    // Vemf is the motor back EMF, Rd is the APEX drive drop resistance and
    // Ra is the motor armature resistance.
    //
    // Note: The computed duty factor is clipped at [-1.5,1.5] to avoid
    // problems when certain values exceed the range required by this model.
    //
    if ( command[index] > 0 ) {

      // Compute the adjusted APEX supply based on desired current
      adjSupply = DRIVE_SUPPLY - DRIVE_DROP * ( command[index] - backEMF ) / RA;
      if ( adjSupply < 0 ) 
        adjSupply = 0;

      if ( 1.5 * adjSupply > command[index] )
        duty = command[index] / adjSupply;
      else
        duty = 1.5;

    } else if ( command[index] < 0 ) {

      // Compute the adjusted APEX supply based on desired current
      adjSupply = DRIVE_SUPPLY + DRIVE_DROP * ( command[index] - backEMF ) / RA;
      if ( adjSupply < 0 ) 
        adjSupply = 0;

      if ( (- 1.5 * adjSupply) < command[index] )
        duty = command[index] / adjSupply;
      else
        duty = - 1.5;

    } else {
      duty = 0.0;
    }
  } else {
    // Motor drive voltage drop compensation is
    // disabled. Straightforward voltage conversion is sufficient.

    duty = command[index] / DRIVE_SUPPLY;
  }

  apexCmd = calib[index].drivePolarity * duty / DRIVE_SCALE;
  apexCmd += calib[index].driveOffset;

  analogIO->write( index, apexCmd );
};

//
// McMotor::computeStates
//
// Motor state estimation. Performs calculations when the motor state
// changes and updates the motor voltage, current and back EMF values.
//
void McMotor::computeStates( uint index ) {

  float duty;
  float scaledSupply, scaledDrop;

  // First, find the corresponding encoder reader if we did not do so
  // already.  This is required, because this module will not have
  // been added during hardware initialization. Moreover, we need the
  // motor speed measurement for the state estimation to work.
  if ( encoder[index] == NULL ) {
    if ( ( encoder[index] = ( EncoderReader * ) 
           MMFindModule( ENCODERREADER_NAME, index )) == NULL)
      MMFatalError ( "McMotor::stateEstimation", 
                     "Cannot find encoder reader!" );

    // Check to see if the encoder reader module is active.
    if ( encoder[index]->getState() != Module::ACTIVE )
      MMWarning( "McMotor::stateEstimation", 
                 "Encoder reader module inactive!" );
  }

  // Now that we have access to the encoder reader, ( which, hopefully
  // is activated ) we can check whether there has been a change in
  // the motor state
  if ( lastSpeed[index] == encoder[index]->getSpeed()
       && analogIO->readOutput( index ) == lastOutput[index] )
    // There is no state change, so previously computed values are
    // still fine.
    return;

  // Otherwise, we can go about performing the necessary calculations.
  lastSpeed[index] = encoder[index]->getSpeed();
  lastOutput[index] = analogIO->readOutput( index );

  // Compute the armature back EMF voltage
  backEMF[index] = backEMFcoeff * lastSpeed[index];
  
  // Compute the PWM duty factor at motor terminals
  duty = calib[index].drivePolarity 
    * ( DRIVE_SCALE * ( lastOutput[index] - calib[index].driveOffset ) );

  // Clip the duty factor value to model drive saturation
  if ( duty > 1.0 ) duty = 1.0;
  if ( duty < -1.0 ) duty = -1.0;

  scaledSupply = duty * DRIVE_SUPPLY;

  /* I took this out on Jun 15 because it caused some unrealistic
   * current estimate problems. -ulucs
   *
  // Figure out what the slope of the motor drive voltage drop should be.
  // Note that if the effective motor drive max voltage is smaller than 
  // the back EMF, then the current will be negative and the negative of 
  // the drive drop resistance should be used.
  if ( scaledSupply > backEMF[index] ) 
    scaledDrop = duty * DRIVE_DROP;
  else
    scaledDrop = - duty * DRIVE_DROP;

  // Find the operating point of the current based on the armature resistance 
  // and the drive voltage drop
  current[index] 
    = ( scaledSupply - backEMF[index] ) / ( RA + scaledDrop );
  */

  // This new approach solves problems with the estimation that the
  // above approach presented.
  scaledDrop = fabs( duty ) * DRIVE_DROP;

  current[index] 
    = ( scaledSupply - backEMF[index] ) / ( RA + scaledDrop );

  // Compute the voltage at motor terminals
  voltage[index] = scaledSupply - current[index] * scaledDrop;
    
}
