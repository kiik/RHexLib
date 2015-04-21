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
 * $Id: VirtualMotor.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The Virtual Motor Module. Simulates a free running DC motor.
 *
 * This file contains the Module class definition, all datatypes
 * and constants associated with the virtual motor module type.
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _VIRTUALMOTOR_HH
#define _VIRTUALMOTOR_HH

#include "basicmath.hh"
#include "ModuleManager.hh"

// Module specific constants ---------------------------------------

// No load motor speed (in rad/s)
#define VM_MAX_SPEED    1047.2
// Stall torque (in Nm)
#define VM_MAX_TORQUE   0.218
// Armature resistance (in ohm )
#define VM_RA           1.33
// Torque constant ( Nm / A )
#define VM_TORQUE_CONST 0.0161
// Speed constant ( rad/s / V )
#define VM_SPEED_CONST  55.3685
// Mechanical time constant (in ms)
#define VM_TIME_CONST   5.0
// Rotor inertia (kg m^2)
#define VM_INERTIA      9.11e-7
// Armature inductance (mH)
#define VM_LA           0.12
// Encoder ratio (count/rev) 
#define VM_ENC_RATIO    2000
// Gear ratio (output rev / shaft rev) 
#define VM_GEAR_RATIO   (1.0 / 33.0)

// Assumed time interval between update() calls (in us)
#define VM_UPDATE_TIME  1000.0

// Motor drive voltage drop coefficient (V/A)
#define VM_DRIVE_DROP   0.0
// Motor drive PWM output  scaling factor (%/V)
#define VM_DRIVE_SCALE  (2.0 / 4.0)
// Motor drive voltage suppy value (V)
#define VM_DRIVE_SUPPLY 18
// Motor drive  voltage offset (V)
#define VM_DRIVE_OFFSET 6

// Analog Output limits (V)
#define VM_AOUT_MIN     0.0
#define VM_AOUT_MAX     10.0

// The VirtualMotor class ----------------------------------------- 

class VirtualMotor : public Module {

private:
  // Position and speed of motor shaft (rad, rad/s)
  double    pos, speed;

  // Armature state
  double    va, ia, backEMF;

  // Origin position for encoder count computation (rad)
  double    basepos;

  // The encoder count
  uint16    encCount;

  // Current voltage at motor terminals
  double    command;

  // Flag indicating encoder status
  bool      encoderEnabled;

public:
  VirtualMotor ( int index );
  
  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void );

  void      enableEncoder( bool enable ) { encoderEnabled = enable; };
  void      resetEncoder( void ) { encCount = 0; basepos = pos; };
  uint16    readEncoder( void ) { return encCount; };

  void      setCommand( double v );
  double    getCommand( void ) { return command; };

  double    getVoltage( void ) { return va; };
  double    getCurrent( void ) { return ia; };
  double    getBackEMF( void ) { return backEMF; };

};

#endif
