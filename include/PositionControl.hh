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
 * $Id: PositionControl.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Motor Position Controller module. 
 *
 * This file contains the Module class definition, all datatypes and
 * constants associated with the motor position controller module
 * type.
 *
 * The motor position controller is a motor control class which
 * achieves PD position control.
 *
 * Created       : Uluc Saranli, 11/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _POSITIONCONTROL_HH
#define _POSITIONCONTROL_HH

// Local includes
#include "ModuleManager.hh"
#include "MotorControl.hh"

class EncoderReader;

// Module specific constants ---------------------------------------

// Data types ------------------------------------------------------

// The PositionControl class ---------------------------------------

class PositionControl : public MotorControl {

public:
  typedef enum { NONE, BACKEMF } CompensationType;

  PositionControl ( int index );

  void init ( void );
  void uninit ( void ) { };
  void activate ( void );
  void deactivate ( void );
  void update ( void );

  // Set the feedthrough torque value.
  void setTorqueOffset( float torque ) { torqueOffset = torque; }

  // The error offset determines the range of the position error in
  // the PD calculations. The range of the error becomes:
  // [-PI+offset,PI+offset] where offset is in [-PI, PI]. Using this
  // feature, it is possible to set the preferred direction of travel
  // for the motor.
  void  setErrorOffset( float offset ) { errorOffset = offset; };
  float getErrorOffset( void ) { return errorOffset; };

  // Choose the type of feedforward compensation term in the feedback loop.
  void setCompensationType( CompensationType type ) { compType = type; };
  CompensationType getCompensationType( void ) { return compType; };

  void freeze( void );   // Sets the motor target to the current motor pos.

private:
  // Modules used by this motor controller

  EncoderReader *enc;

  // Threshold angle for 
  float errorOffset;

  // Explicit torque offset, for things like computed torque control etc.
  float torqueOffset;

  // Current type of feedforward compensation
  CompensationType compType;

  // Some cached computations for efficiency
  float torqueCoeff;   // Coeff for the torque conversion PD control
};


#endif
