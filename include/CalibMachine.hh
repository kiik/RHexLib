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
 * $Id: CalibMachine.hh,v 1.5 2001/08/06 20:22:31 ulucs Exp $
 *
 * The CalibMachine performs motor drive and encoder calibration.
 *
 * Created       : Uluc Saranli, 12/28/2000
 * Last Modified : Uluc Saranli, 07/23/2001
 *
 ********************************************************************/

#ifndef _NEWCALIBMACHINE_HH
#define _NEWCALIBMACHINE_HH

#include "types.hh"
#include "ModuleManager.hh"
#include "StateMachine.hh"

class StallSensor;
class EncoderReader;
class SpeedFilter;
class PositionControl;
class Hardware;

// Module specific constants ---------------------------------------

// Disables polarity calibration and relies on the hardware layer defaults.
#define CALIB_POLARITY_DISABLE true

// Maximum time for calibration attempts ( in seconds )
#define CALIB_TIMEOUT 1

// Voltage command for the seeking behavior and drive calibration (V)
#define CALIB_COMMAND 0.4

// Minimum motor speed for polarity calibration (rad/s)
#define CALIB_POL_MIN_SPEED 0.5
// Minimum time of nonzero rotation for polarity calibration (in seconds )
#define CALIB_POL_TIME 0.05

// Stall timeout for ground encoder calibration
#define CALIB_STALL_TIME 0.2
#define CALIB_STALL_TOLERANCE 1e-3

// Default angle offset for the manual encoder calibration (rad)
#define CALIB_DEF_MANUAL_OFFSET M_PI
// Default angle offset for the ground encoder calibration (rad)
#define CALIB_DEF_GROUND_OFFSET 4.97
// Default angle offset for the switch encoder calibration (rad)
#define CALIB_DEF_SWITCH_OFFSET 4.15

// Calibration type switch ID (ON: Normal calib, OFF: Ground calib )
#define CALIB_TYPE_SWITCH 0
// Automatic calibration switch ID (ON: Switch calib, OFF: Manual calib )
#define CALIB_AUTO_SWITCH 1

class CalibMachine : public StateMachine {
  
public:

  typedef enum { NEGATIVE = -1, NO_POLARITY = 0, POSITIVE = +1 } DrivePolarity;
  typedef enum { CALIBRATING = 0, FAILURE = -1, SUCCESS = 1 } CalibStatus;
  typedef enum { UNSPECIFIED, MANUAL, GROUND, SWITCH } CalibMode;

  CalibMachine ( uint index );
  ~CalibMachine ( void );

  CalibStatus getStatus( void ) { return status; };

  // Invalidates the encoder and drive calibration, causing the machine to
  // perform calibration during the next activation.
  void reset( void );
  void reset( bool polarity, bool encoder, bool drive );

  // Chooses the type of calibration to be used in the next activation.
  void setMode( CalibMode m ) { nextMode = m; };
  CalibMode getMode( void ) { return mode; };

  // Chooses whether the body should be assumed to be upside down or not.
  // If already calibrated, it resets the encoder reader modules to reflect 
  // the change.
  // WARNING: This function should not be called when the motor 
  // controllers are active!
  void setUpside( bool down );
  bool isUpsideDown( void ) { return upsideDownFlag; };

  void init ( void );
  void activate ( void );
  void deactivate ( void );

private:

  // events
  EventObject ( PolInvalidEv ) * polInvalidEv;
  EventObject ( PolValidEv ) * polValidEv;
  EventObject ( StallEv ) * stallEv;
  EventObject ( EncManualEv ) * encManualEv;
  EventObject ( EncGroundEv ) * encGroundEv;
  EventObject ( EncSwitchEv ) * encSwitchEv;
  EventObject ( EncValidEv ) * encValidEv;
  EventObject ( FullRotEv ) * fullRotEv;
  EventObject ( SwitchEv ) * switchEv;
  EventObject ( NoSwitchEv ) * noSwitchEv;
  EventObject ( NoCalibRetryEv ) * noCalibRetryEv;
  EventObject ( CalibRetryEv ) * calibRetryEv;
  EventObject ( DriveInvalidEv ) * driveInvalidEv;
  EventObject ( DriveValidEv ) * driveValidEv;
  EventObject ( LegReadyEv ) * legReadyEv;
  EventObject ( TimeoutEv ) * timeoutEv;

  // states
  StateObject ( PolarityTest ) * polarityTest;
  StateObject ( PolTestPos ) * polTestPos;
  StateObject ( EncoderCalib ) * encoderCalib;
  StateObject ( PolTestNeg ) * polTestNeg;
  StateObject ( CalibFailed ) * calibFailed;
  StateObject ( EncManualCalib ) * encManualCalib;
  StateObject ( EncGroundCalib ) * encGroundCalib;
  StateObject ( EncSwitchCalib ) * encSwitchCalib;
  StateObject ( DriveCalib ) * driveCalib;
  StateObject ( ClearSwitch ) * clearSwitch;
  StateObject ( SeekSwitch ) * seekSwitch;
  StateObject ( SwitchFailed ) * switchFailed;
  StateObject ( PrepareLeg ) * prepareLeg;
  StateObject ( CalibSuccess ) * calibSuccess;
  StateObject ( RefineInterval ) * refineInterval;


  // Internal data --------------------------------

  CalibStatus status;
  CalibMode   mode;
  CalibMode   nextMode;

  double timeoutMark;
  double polarityMark;
  double driveMark;

  bool polarityFlag;   // Is the polarity calibrated?
  bool encoderFlag;    // Is the encoder calibrated?
  bool driveFlag;      // Is the drive offset calibrated?

  // Latest calibration data
  DCMotorHW::MotorCalib_t calib;

  // Various configuration parameters to be read from the symbol table
  double calibTimeout;
  float  seekCommand;
  float  minPolSpeed;
  double polTime;
  double stallTime;
  float  stallTolerance;
  bool   calibRetry;
  int    homeSwitch;   // Index of the home switch for this leg

  // Encoder angle offsets for different types of calibration
  float manualOffset;
  float groundOffset;
  float switchOffset;

  // Flag to choose upside down operation
  bool upsideDownFlag;

  // Some internal stuff
  float lastSpeed;

  // Drive calibration related data
  float posCmd, negCmd, curCmd;

  Hardware        *hardware;
  AnalogHW        *analogIO;
  SwitchHW        *switches;
  DCMotorHW       *dcmotors;
  EncoderReader   *enc;
  SpeedFilter     *spd;
  StallSensor     *stall;
  PositionControl *control;

  void resetSpeed( void );
  bool monitorSpeed( void );

  void stopMotor( void );
  void startMotor( double command );
};

#endif
