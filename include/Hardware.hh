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
 * $Id: Hardware.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Low level hardware interface facilities
 *
 * This file contains all data types and function prototypes for 
 * interfacing with low level hardware. It presents a uniform 
 * interface to standard hardware components such as:
 * - Encoder sensors
 * - Analog outputs
 * - Digital Inputs/Outputs
 * - Analog inputs
 * - Counter/timers
 *
 * Each different hardware port of RHexLib must provide this functionality
 * by deriving new classes from the base classes defined in this file.
 *
 * Created       : Uluc Saranli, 11/16/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _HARDWARE_HH
#define _HARDWARE_HH

#include <stdio.h>
#include "types.hh"

//
// Interface class to 16-bit incremental encoders.
//
class EncoderHW {
public:
  virtual ~EncoderHW( ) { };

  virtual void   enable( uint index ) = 0; // Enable an individual encoder ch.
  virtual void   disable( uint index ) = 0;// Disable an individual encoder ch.

  virtual uint16 read( uint index ) = 0;   // Read the 16 bit value from a ch.
  virtual void   reset( uint index ) = 0;  // Reset the encoder count to zero.
};

//
// Interface class to analog inputs and outputs. All analog values 
// are specified in Volts
//
class AnalogHW {
public:
  virtual ~AnalogHW( ) { };

  virtual float read( uint index ) = 0;  // Return the latest analog read ( V )
  virtual void  write( uint index, float value ) = 0; // Set analog output value ( V )
  virtual void  outputRange( uint index, float *min, float *max ) = 0;
};

//
// Interface class to digital inputs and outputs.  Digital IO bits is
// organized into groups of 8 bits.  The input/output direction of the
// bits are internally set by the hardware layer and cannot be changed
// by this interface.
//
class DigitalHW {
public:
  virtual ~DigitalHW( ) { };

  virtual uint   getByte( uint byte ) = 0;
  virtual void   setByte( uint byte, uint8 value ) = 0;
  virtual bool   getBit( uint byte, uint bit ) = 0;
  virtual void   setBit( uint byte, uint bit, bool value ) = 0;
};

//
// Interface class for 16 bit Intel 82C55 type timers.
//
class TimerHW {
public:
  virtual ~TimerHW( ) { };

  virtual void   setup( uint timer, uint mode, uint32 hertz ) = 0; // Setup timer
  virtual uint16 read( uint timer ) = 0;                           // Read timer count
  virtual void   getInfo( uint timer, uint *initial, uint *clockFreq ) = 0;
};

//
// Interface class for accelerometer hardware.  This interface is
// somewhat independent of the particulars of the hardware. It returns
// m/s^2 measured accelerations ( including the gravitational offset )
// in the sensors frame of reference.
//
class AccelHW {
public:
  virtual ~AccelHW( ) { };

  typedef enum { AXIS_X, AXIS_Y, AXIS_Z } Axis;

  virtual float read( Axis a ) = 0; // Returns the acceleration on an axis (m / s^2)
  virtual void  getInfo( Axis axis, float *resolution, float *limit ) = 0;
};

// Gyro interface class
class GyroHW {
public:
  virtual ~GyroHW( ) { };
    
  typedef enum { AXIS_X, AXIS_Y, AXIS_Z } Axis;
    
    
  virtual float readRate( Axis a ) = 0;  // Returns the angular rates in rads/s
  virtual float readAngle( Axis a ) = 0; // Returns the angle in rads

  virtual void resetAngle( Axis a, float val ) = 0;
  virtual void setOffset( Axis a, float val ) = 0;
    
  virtual void  getInfo( Axis axis, float *resolution, float *limit ) = 0;
};

//
// Interface class for the robot power source.
//
class PowerHW {
public:
  virtual ~PowerHW( ) { };

  virtual float voltage( void ) = 0;  // Measured ( or estimated ) battery voltage
  virtual float current( void ) = 0;  // Measured ( or estimated ) battery current
};

//
// Interface class for ON/OFF switches (such as DIP switches etc.).
//
class SwitchHW {
public:
  virtual ~SwitchHW( ) { };

  virtual bool read( uint index ) = 0;
};

//
// Interface class continuously varying dial inputs ( such as R/C sticks )
// Returns values in the range [-1, 1]
//
class DialHW {
public:
  virtual ~DialHW( ) { };

  virtual float read( uint index ) = 0;
};

//
// Interface class for DC motors and control hardware in the system.
// Each DC motor can be in different control modes (such as voltage mode or
// current mode). In addition to providing measured (or estimated) values for
// motor states, this class should be used for setting the motor terminal 
// voltage(for voltage controlled motors), or current(for current controlled 
// motors).
//
//
class DCMotorHW {
public:
  typedef enum { VOLTAGE_MODE, CURRENT_MODE } ControlMode;
  typedef struct {

    // Motor drive input command corresponding to zero motor rotation (V)
    float driveOffset;
    // Polarity of the motor drive/encoder pair. (-1, 0 or +1 )
    int drivePolarity;

  } MotorCalib_t;

  typedef struct {

    // Motor Torque constant ( Nm / A )
    float torqueConstant;
    // Motor Speed constant ( rad/s / V )
    float speedConstant;
    // Motor Armature resistance ( ohm )
    float terminalResistance;

    // Encoder ratio (count / shaft rev)
    float encRatio;
    // Gear ratio (output rev / shaft rev)
    float gearRatio;

  } MotorParam_t;

  virtual void  getParams( uint index, MotorParam_t *param ) = 0;

  virtual float getVoltage( uint index ) = 0;
  virtual float getCurrent( uint index ) = 0;
  virtual float getBackEMF( uint index ) = 0;
  virtual float getTemperature( uint index ) = 0;

  virtual ControlMode getControlMode( uint index ) = 0;
  virtual void  setCommand( uint index, float cmd ) = 0;
  virtual float getCommand( uint index ) = 0;

  virtual void getCalibration( uint index, MotorCalib_t *calib ) = 0;
  virtual void setCalibration( uint index, MotorCalib_t *calib ) = 0;
  
};

class Hardware {
public:

  Hardware( void ) { 
    encoders = NULL; 
    analogIO = NULL; 
    digitalIO = NULL;
    timers = NULL;
    accels = NULL;
    gyros = NULL;
    power = NULL;
    switches = NULL;
    dials = NULL;
    dcmotors = NULL;
  };
  virtual ~Hardware( ) { };

  // Hardware initialization. Called by MMChooseHardware()
  virtual void initialize( void ) = 0;
  // Hardware initialization. Called by MMShutdown()
  virtual void cleanup( void ) = 0;

  // Returns the current clock value in microseconds since the
  // initialization of the hardware.
  virtual CLOCK readClock( void ) = 0;

  // readUClock() has the same functionality as readClock(), but its
  // resolution is either equal or higher than readClock(). Usually,
  // it uses a slower mechanism to read the clock, so it should be
  // used only when necessary.
  virtual CLOCK readUClock( void ) = 0;

  // Enable or disable motor drives for a particular axis. 
  // Note that some systems cannot support independent enable/disable
  // for different axes.
  virtual void driveEnable( uint index, bool enable ) = 0;

  // Pointers to various components provided by the hardware instance.
  EncoderHW   * encoders;
  AnalogHW    * analogIO;
  DigitalHW   * digitalIO;
  TimerHW     * timers;
  AccelHW     * accels;
  GyroHW      * gyros;
  PowerHW     * power;
  SwitchHW    * switches;
  DialHW      * dials;
  DCMotorHW   * dcmotors;

};

#endif
