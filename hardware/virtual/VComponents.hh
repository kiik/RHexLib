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
 * $Id: VComponents.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Internal data type definitions and function prototypes for the
 * virtual hardware components
 *
 * Created       : Uluc Saranli, 11/16/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _VCOMPONENTS_HH
#define _VCOMPONENTS_HH

#ifdef _QNX4_
#pragma off(unreferenced)
#endif

#include "Hardware.hh"
#include "VirtualHW.hh"
#include "VirtualMotor.hh"

class VEncoder : public EncoderHW {
public:
  ~VEncoder( ) { };

  void   enable( uint index );
  void   disable( uint index );

  uint16 read( uint index );
  void   reset( uint index );

};

class VAnalog : public AnalogHW {
public:
  ~VAnalog( ) { };

  float read( uint index );
  void  write( uint index, float value );

  void  outputRange( uint index, float *min, float *max ) {
    *min = VM_AOUT_MIN;
    *max = VM_AOUT_MAX;
  };

};

// Virtual digital channels. Not implemented yet!.
class VDigital : public DigitalHW {
public:

  uint   getByte( uint byte ) { return 0; };
  void   setByte( uint byte, uint8 value ) { };
  bool   getBit( uint byte, uint bit ) { return false; };
  void   setBit( uint byte, uint bit, bool value ) { };

};

// Virtual timers. Not implemented yet!.
class VTimer : public TimerHW {
public:

  void   setup( uint timer, uint mode, uint32 hertz ) { };
  uint16 read( uint timer ) { return 0; };
  void   getInfo( uint timer, uint *initial, uint *clockFreq ) {
    *initial = 0; *clockFreq = 0; 
  };

};

// Virtual accelerometers. Not implemented yet!.
class VAccel : public AccelHW {
public:

  float read( Axis a ) { return 0.0; };
  void getInfo( Axis axis, float *resolution, float *limit ) {
    *resolution = 1e-3;
    *limit = 2.0;
  };
};

// Virtual gyros. Not implemented yet!.
class VGyro : public GyroHW {
public:

  float readAngle( Axis a ) { return 0.0; };
  float readRate( Axis a ) { return 0.0; };

  void resetAngle( Axis a, float val ) { };
  void setOffset( Axis a, float val ) { };
    
  void getInfo( Axis axis, float *resolution, float *limit ) {
    *resolution = 1e-3;
    *limit = 2.0;
  };
};

// Interface class to the system power (battery etc. )
class VPower : public PowerHW {
public:

  float voltage( void ) { return 24.0; };
  float current( void ) { return 0.0; };
};

class VSwitch : public SwitchHW {
public:

  bool read( uint index );
};

class VDial : public DialHW {
public:

  float read( uint index );
};


// Interface class to the DC motors.
class VMotor : public DCMotorHW {

public:

  VMotor( void );

  void  getParams( uint index, MotorParam_t *param );

  float getVoltage( uint index );
  float getCurrent( uint index );
  float getBackEMF( uint index );
  float getTemperature( uint index ) { return 0.0; };

  ControlMode getControlMode( uint index ) { return controlMode[index]; };
  void setCommand( uint index, float cmd );
  float getCommand( uint index );

  void getCalibration( uint index, MotorCalib_t *calib ) {
    // Virtual device calibration is static, so we can just return
    // constants here.

    calib->driveOffset = VM_DRIVE_OFFSET;
    calib->drivePolarity = +1;

  };

  void setCalibration( uint index, MotorCalib_t *calib ) { };

private:

  ControlMode  controlMode[6];

};

#endif // #ifndef _VCOMPONENTS_HH
