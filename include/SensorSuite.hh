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
 * $Id: SensorSuite.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Various Sensor classes for RHexLib
 *
 * Created       : Uluc Saranli, 01/15/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _SENSORSUITE_HH
#define _SENSORSUITE_HH

// Local includes
#include "GenericSensor.hh"
#include "AnalogOutput.hh"
#include "PulseWidth.hh"
#include "EncoderReader.hh"
#include "SpeedFilter.hh"
#include "Hardware.hh"

// ------------------------------------------------------------------
// AnalogOutSensor class. Reads the current analog output value -----
class AnalogOutSensor : public GenericSensor {

private:
  AnalogOutput *aout;

public:
  AnalogOutSensor( AnalogOutput *m ) { aout = m; };

  float value( void ) { return aout->getValue(); };
};

// ------------------------------------------------------------------
// AnalogInSensor class. Reads the current analog input value -------
class AnalogInSensor : public GenericSensor {

private:
  AnalogHW *aio;
  uint index;

public:
  AnalogInSensor( Hardware *hw, uint i  ) { aio = hw->analogIO; index = i; };

  float value( void ) { return ( aio != NULL ) ? aio->read( index ) : 0.0; };
};

// ------------------------------------------------------------------
// BatteryVoltSensor class. Reads the current battery voltage -------
class BatteryVoltSensor : public GenericSensor {

public:
  BatteryVoltSensor( Hardware *hw  ) { bat = hw->power; };

  float value( void ) { return ( bat != NULL ) ? bat->voltage( ) : 0.0; };

private:
  PowerHW *bat;
};

// ------------------------------------------------------------------
// BatteryCurSensor class. Reads the current battery current --------
class BatteryCurrSensor : public GenericSensor {

public:
  BatteryCurrSensor( Hardware *hw  ) { bat = hw->power; };

  float value( void ) { return ( bat != NULL ) ? bat->current( ) : 0.0; };

private:
  PowerHW *bat;
};

// ------------------------------------------------------------------
// PulseWidthSensor class. Reads the current pulse width value ------
class PulseWidthSensor : public GenericSensor {

public:
  PulseWidthSensor( PulseWidth *m ) { pwm = m; };

  float value( void ) { return pwm->getWidth(); };

private:
  PulseWidth *pwm;
};

// -----------------------------------------------------------------
//  MotorPositionSensor class. Reads the current motor position ----
class MotorPositionSensor : public GenericSensor {

public:
  MotorPositionSensor( EncoderReader *m ) { enc = m; };

  float value( void ) { return enc->getPosition(); };

private:
  EncoderReader *enc;
};

// -----------------------------------------------------------------
//  MotorRawSpeedSensor class. Reads the unfiltered motor speed ----
class MotorRawSpeedSensor : public GenericSensor {

public:
  MotorRawSpeedSensor( EncoderReader *m ) { enc = m; };

  float value( void ) { return enc->getSpeed(); };

private:
  EncoderReader *enc;
};

// -----------------------------------------------------------------
//  MotorSpeedSensor class. Reads the filtered motor speed ---------
class MotorSpeedSensor : public GenericSensor {

public:
  MotorSpeedSensor( SpeedFilter *m ) { filter = m; };

  float value( void ) { return filter->getSpeed(); };

private:
  SpeedFilter *filter;
};

// -----------------------------------------------------------------
// MotorBackEMFSensor class. Reads the motor back EMF estimate -----
class MotorBackEMFSensor : public GenericSensor {

public:
  MotorBackEMFSensor( Hardware *hw, uint i ) { motors = hw->dcmotors; index = i; };

  float value( void ) { return motors->getBackEMF( index ); };

private:
  DCMotorHW *motors;
  uint index;
};

// -----------------------------------------------------------------
// MotorVoltageSensor class. Reads the motor term. voltage estimate 
class MotorVoltageSensor : public GenericSensor {

public:
  MotorVoltageSensor( Hardware *hw, uint i ) { motors = hw->dcmotors; index = i; };

  float value( void ) { return motors->getVoltage( index ); };

private:
  DCMotorHW *motors;
  uint index;
};

// -----------------------------------------------------------------
// MotorCurrentSensor class. Reads the motor term. current estimate 
class MotorCurrentSensor : public GenericSensor {

public:
  MotorCurrentSensor( Hardware *hw, uint i ) { motors = hw->dcmotors; index = i; };

  float value( void ) { return motors->getCurrent( index ); };

private:
  DCMotorHW *motors;
  uint index;
};

// -------------------------------------------------------------------
// MotorTemperatureSensor class. Reads the motor temperature estimate 
class MotorTemperatureSensor : public GenericSensor {

public:
  MotorTemperatureSensor( Hardware *hw, uint i ) { motors = hw->dcmotors; index = i; };

  float value( void ) { return motors->getTemperature( index ); };

private:
  DCMotorHW *motors;
  uint index;
};

// -------------------------------------------------------------------
// DialSensor class. Reads a hardware dial -------------------------- 
class DialSensor : public GenericSensor {

public:
  DialSensor( Hardware *hw, uint i ) { hardware = hw; index = i; };

  float value( void ) { return hardware->dials->read( index ); };

private:
  Hardware *hardware;
  uint index;
};



#endif // #ifndef _SENSORSUITE_HH
