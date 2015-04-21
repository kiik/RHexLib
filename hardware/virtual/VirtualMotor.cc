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
 * $Id: VirtualMotor.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The Virtual Motor Module. Simulates a free running DC motor.
 *
 * This file implements the functions defined in VirtualMotor.hh
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "VirtualMotor.hh"

// VirtualMotor::VirtualMotor : Class constructor
VirtualMotor::VirtualMotor( int index )
  : Module ( "vmotor", index, false, false ) {

  pos = speed = 0.0;

  va = ia = 0.0;

  encCount = 0;

  command = 0.0;

  // Start out with the encoder disabled
  encoderEnabled = false;
}

// VirtualMotor::update : Computes the encoder count and the new shaft position
void VirtualMotor::update( void ) {

  double ss_speed;
  double interval = VM_UPDATE_TIME * 1e-6, tc = (VM_TIME_CONST * 1e-3);
  double twoPi = 2.0 * M_PI;
  int    encDiff;

  // Compute the encoder count if encoders are enabled
  if ( encoderEnabled ) {
    // Convert the shaft pos. in rads to encoder counts
    encDiff = int( ( pos - basepos ) / twoPi * VM_ENC_RATIO );

    // Take the above count MOD 0xffff to get the actual encoder count
    encCount = encDiff - 0xffff * ( encDiff / 0xffff );
  }

  // Update the motor speed based on the current voltage across terminals
  // Compute the armature back EMF voltage
  backEMF = speed / VM_SPEED_CONST;

  // Compute the voltage at motor terminals
  va = command;

  // Compute the armature current
  ia = ( command - backEMF ) / VM_RA;

  // The steady state speed (rad/s)
  ss_speed = va * VM_SPEED_CONST;
  if ( ss_speed > VM_MAX_SPEED ) ss_speed = VM_MAX_SPEED;
  if ( ss_speed < -VM_MAX_SPEED ) ss_speed = -VM_MAX_SPEED;

  // Update the motor position based on the dynamics
  pos = pos +
    tc * (speed - ss_speed)
    - tc * (speed - ss_speed) * exp( - interval / tc )
    + ss_speed * interval;

  // Compute the next value of the speed through integration of the dynamics
  speed = ss_speed
    + (speed - ss_speed) * exp( - interval / tc );

  // printf ( "Motor[%i] pos:%e, spd: %e, enc: %u, cmd: %e\n",
  //         getIndex(), pos, speed, encCount, command );
}

void VirtualMotor::setCommand( double v ) {
  // Clip the voltage to the appropriate range
  command = fmax( fmin( VM_DRIVE_SUPPLY, v ), - VM_DRIVE_SUPPLY );

};
