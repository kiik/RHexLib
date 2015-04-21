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
 * $Id: PositionControl.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Motor position controller module.
 *
 * This file implements the functions defined in PositionControl.hh
 *
 * Created       : Uluc Saranli, 11/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// System includes
#include <stdio.h>
#include <math.h>

// Local includes
#include "StdModules.hh"
#include "MotorControl.hh"
#include "PositionControl.hh"
#include "EncoderReader.hh"

//
//  PositionControl::PositionControl: Class constructor
//
PositionControl::PositionControl( int index )
  : MotorControl ( POSITIONCONTROL_NAME, index ) {

  MotorTarget_t t;
  MotorGains_t g;

  // Set the current target to 0
  t.pos = t.vel = t.acc = 0.0;
  setTarget ( &t );

  // Set the gains to 0
  g.kp = g.kd = 0.0;
  setGains ( &g );

  // Set the torque offset to 0
  setTorqueOffset( 0.0 );

  // Default compensation is BackEMF
  setCompensationType( BACKEMF );

  // The default error range is [-PI, PI], which chooses the closest
  // direction of travel to apply the torque.
  errorOffset = 0;

  enc = NULL;
}

//
// PositionControl::init
//
void PositionControl::init( void ) {

  // Find the encoder module to read motor position
  if ( ( enc = (EncoderReader *) 
         MMFindModule( ENCODERREADER_NAME, getIndex() )) == NULL)
    MMFatalError ( "PositionControl::init", "Cannot find encoder reader!" );

}

//
// PositionControl::activate
//
void PositionControl::activate( void ) {

  // Grab the analog output module

  // Grab and activate the encoder sensor module
  MMGrabModule ( enc, this );

  // Set the targets to a harmless value based on the current mode
  // to avoid an activation step input

  // Set the target to the current motor state
  target.pos = enc->getPosition();
  target.vel = enc->getSpeed();
  target.acc = 0.0;

  // Reread the calibration information. It might have changed in the meantime
  dcmotors->getCalibration( getIndex(), &calib );

  // Precompute some coefficients
  torqueCoeff = motorparams.terminalResistance * motorparams.gearRatio
                  / motorparams.torqueConstant;
}

//
// PositionControl::deactivate
//
void PositionControl::deactivate( void ) {

  // Release the previously grabbed modules 
  MMReleaseModule ( enc, this );
}

//
// PositionControl::update
//
void PositionControl::update( void ) {

  float pos = enc->getPosition();
  float speed = enc->getSpeed();
  float poserr, speederr;
  float twoPi = 2.0 * M_PI;

  float t_d, Vm_d, compensation;
  float goalpos;

  // Compute the current and target angular positions
  pos = pos - floor( pos / twoPi ) * twoPi;
  goalpos = target.pos - floor( target.pos / twoPi ) * twoPi;

  // Compute the error in the angle and speed ignoring the 2PI wraparound
  poserr = goalpos - pos;
  speederr = target.vel - speed;

  // Correct the position error to respect the allowable range.
  //
  // The range is determined by the error offset parameter:
  // range = [-PI+offset,PI+offset].
  //
  // Note that this also takes care of the 2*PI wraparound problem in
  // the angle.
  //
  if ( poserr <= - M_PI + errorOffset )
    poserr += twoPi;

  if ( poserr > M_PI + errorOffset )
    poserr -= twoPi;

  // Compute the desired torque command
  t_d = gains.kp * (poserr) + gains.kd * (speederr) + torqueOffset;

  // Based on the control mode of the underlying hardware, compute the
  // motor terminal voltage to output.
  if ( dcmotors->getControlMode( getIndex() ) 
       == DCMotorHW::VOLTAGE_MODE ) {   

    if ( compType == BACKEMF ) {
      // Get the back EMF estimation from the current hardware.
      compensation = dcmotors->getBackEMF( getIndex() );

    } else if ( compType == NONE ) {

      // No feedforward compensation
      compensation = 0.0;
    }

    // There are two options in the feedback control. One uses
    // target.vel to compute the feedforward compensation for the
    // control. The alternative is to use the current speed to
    // compensate for the motor back EMF, obtaining approximate torque
    // control, which is closer to the system dynamics than the first
    // version. Th second option seems to give better performance, but
    // we need to look into this a little bit more carefully. - Uluc.

    // Compute the desired terminal voltage based on the desired torque and
    // compensating for the current back EMF.
    Vm_d = t_d * torqueCoeff + compensation;

    // Let the hardware layer deal with the conversion to command whatever
    // motor drives are present in the current hardware.
    dcmotors->setCommand( getIndex(), Vm_d );

  } else {
    dcmotors->setCommand( getIndex(), t_d );
  }
 
}

//
// PositionControl::freeze : Sets the current target to the current motor position
//
void PositionControl::freeze( void ) {

  MotorTarget_t target;

  // Retrieve the current position
  target.pos = enc->getPosition();
  target.vel = 0.0;
  target.acc = 0.0;

  // Set the control target position.
  setTarget( &target );
}
