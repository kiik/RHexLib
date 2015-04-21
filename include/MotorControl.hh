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
 * $Id: MotorControl.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Motor Controller module. 
 *
 * This file contains the Module class definition, all datatypes
 * and constants associated with the motor controller module type.
 * The motor controller achieves motor control in one of either 
 * PD position, orspeed or torque control.
 *
 * Created       : Uluc Saranli, 10/27/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _MOTORCONTROL_HH
#define _MOTORCONTROL_HH

#include "ModuleManager.hh"
#include "Hardware.hh"

// Module specific constants ---------------------------------------

// Data types ------------------------------------------------------

// Struct to hold the target setting information for a motor
typedef struct {

  // The setpoint and its derivatives
  double    pos;
  double    vel;
  double    acc;

} MotorTarget_t;

// Struct to hold controller gains
typedef struct {

  // Proportional and derivative gains
  double        kp;
  double        kd;
  double        ka;

} MotorGains_t;

// The MotorControl class -----------------------------------------

class MotorControl : public Module {
public:
  MotorControl( char *name, int index )
    : Module ( name, index, true, false ) {

    hardware = MMGetHardware();
    if ( ( dcmotors = hardware->dcmotors ) == NULL )
      MMFatalError( "MotorControl::MotorControl",
                    "DCMotorHW component is not supported!" );

    target.pos = target.vel = target.acc = 0.0;
    gains.kp = gains.kd = 0.0;
    dcmotors->getParams( getIndex(), &motorparams );
    dcmotors->getCalibration( getIndex(), &calib );
  }

  virtual void init ( void ) = 0;
  virtual void uninit ( void ) = 0;
  virtual void activate ( void ) = 0;
  virtual void deactivate ( void ) = 0;
  virtual void update ( void ) = 0;

  void getGains ( MotorGains_t *g ) { *g = gains; };
  void setGains ( MotorGains_t *g ) { gains = *g; };
  void getTarget ( MotorTarget_t *t ) { *t = target; };
  void setTarget ( MotorTarget_t *t ) { target = *t; };

protected:

  // Low level hardware interface
  Hardware      *hardware;
  DCMotorHW     *dcmotors;

  // Target setpoint
  MotorTarget_t target;

  // Control gains
  MotorGains_t  gains;

  // Hardware parameters
  DCMotorHW::MotorParam_t motorparams;

  // Hardware calibration
  DCMotorHW::MotorCalib_t calib;

};


#endif
