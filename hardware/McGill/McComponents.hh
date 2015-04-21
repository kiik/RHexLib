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
 * $Id: McComponents.hh,v 1.7 2001/08/12 16:04:23 ulucs Exp $
 *
 * Internal data type definitions and function prototypes for the
 * hardware components of the University of McGill version of 
 * RHex.
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _MCCOMPONENTS_HH
#define _MCCOMPONENTS_HH

// Global variables for general access
extern class dm6814 *DM6814Card[2];
extern class mpc550 *MPC550Card;

#include "config.hh"
#include "ModuleManager.hh"
#include "Hardware.hh"
#include "McGillHW.hh"
#include "io/dm6814.hh"
#include "io/mpc550.hh"
#include "McAnalogProbe.hh"
#include "EncoderReader.hh"

// --------------------------------------------------------------------------
// Interface class to the encoder chips -------------------------------------
//
// There are 6 encoders in RHex accessed through this class, which reside
// in two separate DM6814 cards. The index parameter is interpreted as
// follows:
//
// index 0-2 : DM6814 #1 encoders 0-2
// index 3-5 : DM6814 #2 encoders 0-2

class McEncoder : public EncoderHW {
public:
  McEncoder( void );
  ~McEncoder( );

  void   enable( uint index );
  void   disable( uint index );

  uint16 read( uint index );
  void   reset( uint index );

};

// McEncoder::read : Reads the current encoder count for the selected channel.
// Note that this procedure selects the appropriate card for the channel
inline uint16 McEncoder::read( uint index ) {

  if ( index < 3 )
    return DM6814Card[0]->readEncoder( index );
  else if ( index < 6 )
    // The right side motor directions are reversed
    return uint16( 0xffff - DM6814Card[1]->readEncoder( index - 3 ) + 0x0001 );
  else {
    // This should never happen. We only have 6 encoders!
    MMWarning( "MEncoder::read", "Encoder index out of bounds!" );
    return 0;
  }
}

// --------------------------------------------------------------------------
// Interface class to the Analog IO channels --------------------------------
//
// There are a total of 8 analog outputs and 8 analog inputs in RHex, all 
// in the MPC550 IO card. The index parameters selects the analog input
// or output channel.

class McAnalog : public AnalogHW {
private:
  float outputCoeff;
  float output[6];
  
public:
  McAnalog( void );
  ~McAnalog( );
  McAnalogProbe *probe;
    
  float read( uint index );
  void  write( uint index, float value );

  void  outputRange( uint index, float *min, float *max ) {
    index = index; // This avoids an annoying compiler warning
    *min = AOUT_MIN;
    *max = AOUT_MAX;
  };

  // Method for accessing the latest output.
  float readOutput( uint index ) { return output[index]; };
};

// McAnalog::write : Writes a DAC value
inline void McAnalog::write( uint index, float value ) {
  // Convert Volts to DAC value
  output[index] = value;
  MPC550Card->analogOut( index, uint16( (value - AOUT_MIN) * outputCoeff ) );
}

// McAnalog::read : Rea analog inputs.
inline float McAnalog::read( uint index ) {
  return probe->getValue( index );
}

// --------------------------------------------------------------------------
// Interface class to the digital IO channels -------------------------------
//
// There are a total of 72 digital channels in RHex. These channels are organized
// into 9 bytes of 8 bits each. The allocation is as follows:
//
// Byte 0: bits 7-0: (MPC550 Port A)
// Byte 1: bits 7-0: (MPC550 Port B)
// Byte 2: bits 7-0: (MPC550 Port C)
// Byte 3: bits 7-0: (DM6814 #1 chip #1)
// Byte 4: bits 7-0: (DM6814 #1 chip #2)
// Byte 5: bits 7-0: (DM6814 #1 chip #3)
// Byte 6: bits 7-0: (DM6814 #2 chip #1)
// Byte 7: bits 7-0: (DM6814 #2 chip #2)
// Byte 8: bits 7-0: (DM6814 #2 chip #3)

class McDigital : public DigitalHW {
public:
  McDigital( void );
  ~McDigital( );

  uint   getByte( uint byte );
  void   setByte( uint byte, uint8 value );
  bool   getBit( uint byte, uint bit );
  void   setBit( uint byte, uint bit, bool value );

};

// --------------------------------------------------------------------------
// Interface class to the Timer channels ------------------------------------
//
// There are a total of 9 timer channels in RHex, 3 in each DM6814 
// and 3 in the MPC550 card. The index parameter is interpreted as follows:
//
// index 0-2 : DM6814 #1 timers 0-2
// index 3-5 : DM6814 #2 timers 0-2
// index 6-8 : MPC550 timers 0-2

class McTimer : public TimerHW {
public:
  McTimer( void );
  ~McTimer( );

  void   setup( uint timer, uint mode, uint32 hertz );
  uint16 read( uint timer );
  void   getInfo( uint timer, uint *initial, uint *clockFreq );

};

// MTimer::read : Reads the current count of a particular timer.
inline uint16 McTimer::read( uint timer ) {
  switch ( timer ) {
  case 0:
  case 1:
  case 2:
    return DM6814Card[0]->readTimer( timer );

  case 3:
  case 4:
  case 5:
    return DM6814Card[1]->readTimer( timer - 3 );

  case 6:
  case 7:
  case 8:
    return MPC550Card->readTimer( timer - 6 );

  default:
    // Warning!: Invalid timer id. SHould not happen!
    return 0;
  }
}


// --------------------------------------------------------------------------
// Interface to the accelerometers. -----------------------------------------

class McAccel : public AccelHW {

public:
  McAccel( Hardware *hw );
  ~McAccel( );

  float read( Axis axis );
  void  getInfo( Axis axis, float *resolution, float *limit );

private:

  // Affine offsets and gains for the accelerometers, mapping the PWM width
  // to the acceleration value. Read from symbol table
  float offsets[3];
  float gains[3];
  Hardware *hw;
    
};

// Interface to the gyros. ---------------------------------------------

class McGyro : public GyroHW {

public:
    McGyro( Hardware *hw );
    ~McGyro(  );

    void resetAngle(Axis axis, float val);
    void setOffset(Axis axis, float val);
    
    float readRate( Axis axis ); // returns angular rate in rads/s:
    float readAngle(Axis axis ); // returns angle in rads:

    void  getInfo( Axis axis, float *resolution, float *limit );

    void update();

private:

    float offsets[3];
    float gains[3];
    float rates[3];
    float oldRates[3];
    float angles[3];
    float lastTime;
    Hardware *hw;
};

// --------------------------------------------------------------------------
// Interface class to the system power (battery etc. ) ----------------------

class McPower : public PowerHW {
public:
  McPower( Hardware *hw );

  float voltage( void );
  float current( void );

private:
  AnalogHW *analogIO;

  float vScale, vOffset; // Scale and offset for conversion to battery voltage
  float iScale, iOffset; // Scale and offset for conversion to battery current
};

inline float McPower::voltage( void ) {
  return ( vOffset + vScale * analogIO->read( BATT_V_CHANNEL ) );
}

inline float McPower::current( void ) {
  return ( iOffset + iScale * analogIO->read( BATT_I_CHANNEL ) );
}


// --------------------------------------------------------------------------
// Interface class to digital DIP switches. ---------------------------------
//
// All 8 switches in McGill RHex are connected to digital IO byte 0
//

class McSwitch : public SwitchHW {

public:
  McSwitch( Hardware *hw ) { digitalIO = hw->digitalIO; };

  // Note that the digital input is inverted because of the way the DIP
  // switches are connected
  bool read( uint index ) { 
    if (index <8 )
      return bool(!digitalIO->getBit( 1, (7 - index) & 0x07 )); 
    else
      switch(index) {
      case 8:
        return bool(digitalIO->getBit(3, 2));
      case 9:
        return bool(digitalIO->getBit(4, 2));
      case 10:
        return bool(digitalIO->getBit(5, 2));
      case 11:
        return bool(digitalIO->getBit(6, 2));
      case 12:
        return bool(digitalIO->getBit(7, 2));
      case 13:
        return bool(digitalIO->getBit(8, 2));
      }
    return false;
  }

private:
  DigitalHW *digitalIO;

};

// --------------------------------------------------------------------------
// Interface class to continuously variable dials. --------------------------
//
// In Michigan RHex, all 6 dials are connected to RC channels
//

class McDial : public DialHW {
public:
  McDial( Hardware *hw );
  ~McDial( );

  float read( uint index );
};

// --------------------------------------------------------------------------
// Interface class to the DC motors. ----------------------------------------

class McMotor : public DCMotorHW {

public:

  McMotor( Hardware *hw );

  void  getParams( uint index, MotorParam_t *param );

  float getVoltage( uint index );
  float getCurrent( uint index );
  float getBackEMF( uint index );
  float getTemperature( uint index ) { return temperature[index]; };

  ControlMode getControlMode( uint index ) { return controlMode[index]; };
  void setCommand( uint index, float cmd );
  float getCommand( uint index ) { return command[ index ]; };

  void getCalibration( uint ind, MotorCalib_t *c ) { 
    if ( ind < 6 ) *c = calib[ ind ]; 
  };
  void setCalibration( uint ind, MotorCalib_t *c ) { 
    if ( ind < 6 ) calib[ ind ] = *c; 
  };

private:

  McAnalog      *analogIO;       // Access pointers to various other components
  EncoderReader *encoder[6];

  ControlMode   controlMode[6];  // The control mode of the underlying hw.
  float         command[6];      // Latest issued voltage command

  // Flag to indicate whether state estimation will be used
  bool stateEstimation;
  // Flag to indicate whether the voltage drop on the motor drives
  // should be compensated for or not
  bool driveCompensation;

  // calibration data for each of the motors
  MotorCalib_t calib[6];

  // Latest state estimation computations
  float voltage[6], current[6], backEMF[6], temperature[6];
  float lastSpeed[6], lastOutput[6];
  float backEMFcoeff;

  void computeStates( uint index );  // Performs motor state estimations
};

#endif // #ifndef _MCCOMPONENTS_HH
