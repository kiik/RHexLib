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
 * $Id: CalibMachine.cc,v 1.4 2001/08/06 20:22:31 ulucs Exp $
 *
 * Created       : Uluc Saranli, 07/23/2001
 * Last Modified : Uluc Saranli, 07/23/2001
 *
 ********************************************************************/

#include <math.h>
#include "StdModules.hh"
#include "ModuleManager.hh"
#include "Hardware.hh"
#include "CalibMachine.hh"
#include "EncoderReader.hh"
#include "SpeedFilter.hh"
#include "StallSensor.hh"
#include "PositionControl.hh"

#define OWNER ( ( CalibMachine * ) owner )

#define DEBUG_MSG( str ) // { char msg[128]; sprintf( msg, "%i: ", OWNER->getIndex() ); MMMessage( msg ); MMMessage( str ); }

// ===================================================================
// Events
// ===================================================================
bool CalibMachine::PolInvalidEv::check ( void ) { 

  return bool( ! OWNER->polarityFlag );
}

bool CalibMachine::PolValidEv::check ( void ) { 

  return  bool( OWNER->polarityFlag );
}

bool CalibMachine::StallEv::check ( void ) { 

  return bool( OWNER->stall->read() ); 
};

bool CalibMachine::EncManualEv::check ( void ) { 

  return bool( ! OWNER->encoderFlag && OWNER->mode 
               == CalibMachine::MANUAL );
}

bool CalibMachine::EncGroundEv::check ( void ) { 

  return bool( ! OWNER->encoderFlag && OWNER->mode 
               == CalibMachine::GROUND );
}

bool CalibMachine::EncSwitchEv::check ( void ) { 

  return bool( ! OWNER->encoderFlag && OWNER->mode 
               == CalibMachine::SWITCH );
}

bool CalibMachine::EncValidEv::check ( void ) { 

  return bool( OWNER->encoderFlag );
}

bool CalibMachine::FullRotEv::check ( void ) { 

  return bool ( (fabs( OWNER->enc->getRelPosition() ) > 2.0 * M_PI )? 1 : 0 );
}

bool CalibMachine::SwitchEv::check ( void ) { 

  return OWNER->switches->read( OWNER->homeSwitch );
}

bool CalibMachine::NoSwitchEv::check ( void ) { 

  return bool( ! OWNER->switches->read( OWNER->homeSwitch ) );
}

bool CalibMachine::NoCalibRetryEv::check ( void ) { 

  return bool( ! OWNER->calibRetry ); 
}

bool CalibMachine::CalibRetryEv::check ( void ) { 

  return bool( OWNER->calibRetry ); 
}

bool CalibMachine::DriveInvalidEv::check ( void ) { 

  return bool( ! OWNER->driveFlag );
}

bool CalibMachine::DriveValidEv::check ( void ) { 

  return bool( OWNER->driveFlag );
}

bool CalibMachine::LegReadyEv::check ( void ) { 

  return bool ( ( OWNER->enc->getPosition() > M_PI - 0.2 )
                || ( OWNER->enc->getPosition() < - M_PI + 0.2 ) );
}


bool CalibMachine::TimeoutEv::check ( void ) { 

  return bool( MMReadTime() >= OWNER->timeoutMark + OWNER->calibTimeout );
}


// States ------------------------------------------------------------

// ============================
// PolarityTest
// ============================
void CalibMachine::PolarityTest::entry ( void ) {

  DEBUG_MSG("CalibMachine::PolarityTest::entry\n");

  OWNER->status = CALIBRATING;
}

void CalibMachine::PolarityTest::during ( void ) {}
void CalibMachine::PolarityTest::exit ( void ) {}

// ============================
// PolTestPos
// ============================
void CalibMachine::PolTestPos::entry ( void ) {

  DEBUG_MSG("CalibMachine::PolTestPos::entry\n");

  // Reset the speed measurement tool
  OWNER->resetSpeed();

  // Output a small voltage in the positive direction
  OWNER->startMotor( OWNER->seekCommand );
}

void CalibMachine::PolTestPos::during ( void ) {

  if ( OWNER->monitorSpeed() ) {
    // Speed measurement successful, perform polarity calibration

    if ( OWNER->spd->getSpeed() > 0 )

      OWNER->calib.drivePolarity = POSITIVE;

    else

      OWNER->calib.drivePolarity = NEGATIVE;

    OWNER->dcmotors->setCalibration( OWNER->getIndex(), &OWNER->calib );

    OWNER->polarityFlag = true;
  }
}

void CalibMachine::PolTestPos::exit ( void ) {

  DEBUG_MSG("CalibMachine::PolTestPos::exit\n");

  // Stop motor rotation
  OWNER->stopMotor();
}

// ============================
// PolTestNeg
// ============================
void CalibMachine::PolTestNeg::entry ( void ) {

  DEBUG_MSG("CalibMachine::PolTestNeg::entry\n");

  // Reset the speed measurement tool.
  OWNER->resetSpeed();

  // Start the motor in the negative direction
  OWNER->startMotor( - OWNER->seekCommand );
}

void CalibMachine::PolTestNeg::during ( void ) {

  if ( OWNER->monitorSpeed() ) {
    // Speed measurement successful, perform polarity calibration
    if ( OWNER->spd->getSpeed() > 0 )

      OWNER->calib.drivePolarity = NEGATIVE;

    else

      OWNER->calib.drivePolarity = POSITIVE;

    OWNER->dcmotors->setCalibration( OWNER->getIndex(), &OWNER->calib );

    OWNER->polarityFlag = true;
  }
}

void CalibMachine::PolTestNeg::exit ( void ) {
  DEBUG_MSG("CalibMachine::PolTestNeg::exit\n");

  // Stop motor rotation
  OWNER->stopMotor();
}

// ============================
// EncoderCalib
// ============================
void CalibMachine::EncoderCalib::entry ( void ) {

  DEBUG_MSG("CalibMachine::EncoderCalib::entry\n");

  // Grab the encoder reader. The Calibmachine is responsible from
  // keeping the encoder reader active as long as the calibration is
  // valid
  if ( !OWNER->encoderFlag )
    MMGrabModule ( OWNER->enc, OWNER );
}

void CalibMachine::EncoderCalib::during ( void ) {}
void CalibMachine::EncoderCalib::exit ( void ) {}

// ============================
// EncManualCalib
// ============================
void CalibMachine::EncManualCalib::entry ( void ) {
  DEBUG_MSG("CalibMachine::EncManualCalib::entry\n");

  // Calibrate the encoder's base value
  OWNER->enc->reset( OWNER->manualOffset );
  OWNER->encoderFlag = true;
}
void CalibMachine::EncManualCalib::during ( void ) {}
void CalibMachine::EncManualCalib::exit ( void ) {}

// ============================
// EncGroundCalib
// ============================
void CalibMachine::EncGroundCalib::entry ( void ) {
  DEBUG_MSG("CalibMachine::EncGroundCalib::entry\n");

  // Output a small voltage to start motor rotation
  OWNER->startMotor( OWNER->calib.drivePolarity * OWNER->seekCommand );

  // Reset the relative position counter of EncoderReader
  OWNER->enc->resetRelPosition();

  // Reset the StallSensor for fresh stall detection.
  OWNER->stall->reset();
}
void CalibMachine::EncGroundCalib::during ( void ) {

  if ( OWNER->stallEv->check() ) {
    // Ground detection successful, perform calibration. Otherwise,
    // leave the motor uncalibrated.

    OWNER->enc->reset( OWNER->groundOffset );
    OWNER->encoderFlag = true;
  }
}
void CalibMachine::EncGroundCalib::exit ( void ) {
  DEBUG_MSG("CalibMachine::EncGroundCalib::exit\n");

  OWNER->stopMotor();

  if ( ! OWNER->stallEv->check() ) {
    // Ground calibration failed. Release the EncoderReader module.

    MMReleaseModule ( OWNER->enc, OWNER );
  }
}

// ============================
// EncSwitchCalib
// ============================
void CalibMachine::EncSwitchCalib::entry ( void ) {
  DEBUG_MSG("CalibMachine::EncSwitchCalib::entry\n");

  // Output a small voltage to start positive motor rotation
  OWNER->startMotor( OWNER->calib.drivePolarity * OWNER->seekCommand );

  // Reset the relative position counter of EncoderReader
  OWNER->enc->resetRelPosition();

  // Reset the StallSensor for fresh stall detection.
  OWNER->stall->reset();
}
void CalibMachine::EncSwitchCalib::during ( void ) {}
void CalibMachine::EncSwitchCalib::exit ( void ) {}

// ============================
// ClearSwitch
// ============================
void CalibMachine::ClearSwitch::entry ( void ) {
  DEBUG_MSG("CalibMachine::ClearSwitch::entry\n");

  // Reset the relative position counter of EncoderReader
  OWNER->enc->resetRelPosition();

}
void CalibMachine::ClearSwitch::during ( void ) { }
void CalibMachine::ClearSwitch::exit ( void ) {
  DEBUG_MSG("CalibMachine::ClearSwitch::exit\n");
}

// ============================
// SeekSwitch
// ============================
void CalibMachine::SeekSwitch::entry ( void ) {
  DEBUG_MSG("CalibMachine::SeekSwitch::entry\n");

  // Output a small voltage to start negative motor rotation
  OWNER->startMotor( - OWNER->calib.drivePolarity * OWNER->seekCommand );

  // Reset the relative position counter of EncoderReader
  OWNER->enc->resetRelPosition();

  // Reset the StallSensor for fresh stall detection.
  OWNER->stall->reset();
}
void CalibMachine::SeekSwitch::during ( void ) {

  if ( OWNER->switchEv->check() ) {
    // Switch detection successful.Perform encoder calibration.
    // Otherwise, leave the encoder uncalibrated.

    OWNER->enc->reset( OWNER->switchOffset );
    OWNER->encoderFlag = true;
  }
}

void CalibMachine::SeekSwitch::exit ( void ) {

  DEBUG_MSG("CalibMachine::SeekSwitch::exit\n");

  // Stop the motor
  OWNER->stopMotor();
}

// ============================
// SwitchFailed
// ============================
void CalibMachine::SwitchFailed::entry ( void ) {

  DEBUG_MSG("CalibMachine::SwitchFailed::entry\n");

  // Stop the motor
  OWNER->stopMotor();
}
void CalibMachine::SwitchFailed::during ( void ) {}
void CalibMachine::SwitchFailed::exit ( void ) {}

// ============================
// DriveCalib
// ============================
void CalibMachine::DriveCalib::entry ( void ) {}
void CalibMachine::DriveCalib::during ( void ) {}
void CalibMachine::DriveCalib::exit ( void ) {}

// ============================
// PrepareLeg
// ============================
void CalibMachine::PrepareLeg::entry ( void ) {

  MotorGains_t  gains;
  MotorTarget_t target;

  DEBUG_MSG("CalibMachine::PrepareLeg::entry\n");

  gains.kp = 9.606;
  gains.kd = 0.72045;
  target.pos = M_PI;
  target.vel = 0.0;
  target.acc = 0.0;

  // Grab the position controller module
  MMGrabModule( OWNER->control, OWNER );

  // Set the controller gains to some appropriate value
  OWNER->control->setGains ( &gains );
  OWNER->control->setTarget ( &target );

  // Reset the timeout Mark for measureing time spent in this state
  OWNER->timeoutMark = MMReadTime();
}
void CalibMachine::PrepareLeg::during ( void ) {}
void CalibMachine::PrepareLeg::exit ( void ) {
  DEBUG_MSG("CalibMachine::PrepareLeg::exit\n");

  // Release and deactivate the position controller module
  MMReleaseModule( OWNER->control, OWNER );

  // Stop motor rotation
  OWNER->stopMotor( );
}

// ============================
// RefineInterval
// ============================
void CalibMachine::RefineInterval::entry ( void ) {

  DEBUG_MSG("CalibMachine::RefineInterval::entry\n");

  // Initialize the two ends of the interval
  OWNER->posCmd = OWNER->calib.drivePolarity * OWNER->seekCommand;
  OWNER->negCmd = - OWNER->calib.drivePolarity * OWNER->seekCommand;

  // Output the mid-value to the analog channel to start motor rotation
  OWNER->curCmd = ( OWNER->posCmd + OWNER->negCmd ) / 2.0;
  OWNER->startMotor( OWNER->curCmd );

  // Record the current time for both speed measurement and timeout check.
  OWNER->timeoutMark = OWNER->driveMark = MMReadTime();

}

void CalibMachine::RefineInterval::during ( void ) {

  double now = MMReadTime();
  double speed;

  if ( now - OWNER->driveMark > OWNER->polTime ) {
    // Enough time has elapsed, we can read the speed measurement
    speed = OWNER->spd->getSpeed();

    // If the speed was small enough, we are done.
    if ( fabs( speed ) < OWNER->stallTolerance ) {

      OWNER->calib.driveOffset += OWNER->curCmd;
      OWNER->dcmotors->setCalibration( OWNER->getIndex(), &OWNER->calib );
      OWNER->driveFlag = true;

    } else {
      // Update the boundaries
      if ( speed > 0.0 )

        OWNER->posCmd = OWNER->curCmd;

      else

        OWNER->negCmd = OWNER->curCmd;
    }

    // Finish the drive calibration if the pos and neg commands are too close
    if ( fabs( OWNER->posCmd - OWNER->negCmd ) < 1e-2 ) {

      OWNER->calib.driveOffset += OWNER->posCmd;
      OWNER->dcmotors->setCalibration( OWNER->getIndex(), &OWNER->calib );
      OWNER->driveFlag = true;

    }

    // Compute the new command as the midpoint of the positive and
    // negative commands
    OWNER->curCmd = ( OWNER->posCmd + OWNER->negCmd ) / 2.0;

    // Output the mid-value to the analog channel to start motor rotation
    OWNER->startMotor( OWNER->curCmd );
    OWNER->driveMark = MMReadTime();

    //  printf("Leg %i: Updated interval:[%f, %f], %f\n", OWNER->getIndex(),
    //         OWNER->negCmd, OWNER->posCmd, speed);
  }
}
void CalibMachine::RefineInterval::exit ( void ) {

  DEBUG_MSG("CalibMachine::RefineInterval::exit\n");

  // Stop motor rotation
  OWNER->stopMotor();

}

// ============================
// CalibFailed
// ============================
void CalibMachine::CalibFailed::entry ( void ) {
  //  char msg[128];

  DEBUG_MSG("CalibMachine::CalibFailed::entry\n");

  // Disable motor drives
  OWNER->hardware->driveEnable( OWNER->getIndex(), false );

  //  sprintf( msg, "Motor %i calibration failed!\n", OWNER->getIndex() );
  //  MMMessage( msg );

  OWNER->status = FAILURE;

}
void CalibMachine::CalibFailed::during ( void ) {}
void CalibMachine::CalibFailed::exit ( void ) {}

// ============================
// CalibSuccess
// ============================
void CalibMachine::CalibSuccess::entry ( void ) {

  //  char msg[128];

  //  sprintf( msg, "Motor %i successfully calibrated.\n", OWNER->getIndex() );
  //  MMMessage( msg );

  DEBUG_MSG("CalibMachine::CalibSuccess::entry\n");

  OWNER->status = SUCCESS;

  // Disable motor drives
  OWNER->hardware->driveEnable( OWNER->getIndex(), false );
}
void CalibMachine::CalibSuccess::during ( void ) {}
void CalibMachine::CalibSuccess::exit ( void ) {}

// =========================================================================
// NewCalibmachine public methods
// =========================================================================

CalibMachine::CalibMachine ( uint index ) 
  : StateMachine ( CALIBMACHINE_NAME, index ) {

  // allocate events
  polInvalidEv = new PolInvalidEv ( this, "PolInvalidEv" );
  polValidEv = new PolValidEv ( this, "PolValidEv" );
  stallEv = new StallEv ( this, "StallEv" );
  encManualEv = new EncManualEv ( this, "EncManualEv" );
  encGroundEv = new EncGroundEv ( this, "EncGroundEv" );
  encSwitchEv = new EncSwitchEv ( this, "EncSwitchEv" );
  encValidEv = new EncValidEv ( this, "EncValidEv" );
  fullRotEv = new FullRotEv ( this, "FullRotEv" );
  switchEv = new SwitchEv ( this, "SwitchEv" );
  noSwitchEv = new NoSwitchEv ( this, "NoSwitchEv" );
  noCalibRetryEv = new NoCalibRetryEv ( this, "NoCalibRetryEv" );
  calibRetryEv = new CalibRetryEv ( this, "CalibRetryEv" );
  driveInvalidEv = new DriveInvalidEv ( this, "DriveInvalidEv" );
  driveValidEv = new DriveValidEv ( this, "DriveValidEv" );
  legReadyEv = new LegReadyEv ( this, "LegReadyEv" );
  timeoutEv = new TimeoutEv ( this, "TimeoutEv" );

  // allocate states
  polarityTest = new PolarityTest ( this, "polarityTest" );
  polTestPos = new PolTestPos ( this, "polTestPos" );
  encoderCalib = new EncoderCalib ( this, "encoderCalib" );
  polTestNeg = new PolTestNeg ( this, "polTestNeg" );
  calibFailed = new CalibFailed ( this, "calibFailed" );
  encManualCalib = new EncManualCalib ( this, "encManualCalib" );
  encGroundCalib = new EncGroundCalib ( this, "encGroundCalib" );
  encSwitchCalib = new EncSwitchCalib ( this, "encSwitchCalib" );
  driveCalib = new DriveCalib ( this, "driveCalib" );
  clearSwitch = new ClearSwitch ( this, "clearSwitch" );
  seekSwitch = new SeekSwitch ( this, "seekSwitch" );
  switchFailed = new SwitchFailed ( this, "switchFailed" );
  prepareLeg = new PrepareLeg ( this, "prepareLeg" );
  calibSuccess = new CalibSuccess ( this, "calibSuccess" );
  refineInterval = new RefineInterval ( this, "refineInterval" );

  // transitions ( in the form < From, Event, To > )
  Transition ( polarityTest, polInvalidEv, polTestPos );
  Transition ( polarityTest, polValidEv, encoderCalib );
  Transition ( polTestPos, stallEv, polTestNeg );
  Transition ( polTestPos, polValidEv, encoderCalib );
  Transition ( polTestNeg, polValidEv, encoderCalib );
  Transition ( polTestNeg, stallEv, calibFailed );
  Transition ( encoderCalib, encManualEv, encManualCalib );
  Transition ( encoderCalib, encGroundEv, encGroundCalib );
  Transition ( encoderCalib, encSwitchEv, encSwitchCalib );
  Transition ( encoderCalib, encValidEv, driveCalib );
  Transition ( encManualCalib, encValidEv, driveCalib );
  Transition ( encGroundCalib, encValidEv, driveCalib );
  Transition ( encGroundCalib, fullRotEv, calibFailed );
  Transition ( encSwitchCalib, switchEv, clearSwitch );
  Transition ( encSwitchCalib, stallEv, seekSwitch );
  Transition ( encSwitchCalib, fullRotEv, switchFailed );
  Transition ( clearSwitch, noSwitchEv, seekSwitch );
  Transition ( clearSwitch, fullRotEv, switchFailed );
  Transition ( seekSwitch, fullRotEv, switchFailed );
  Transition ( seekSwitch, stallEv, switchFailed );
  Transition ( seekSwitch, encValidEv, driveCalib );
  Transition ( switchFailed, noCalibRetryEv, calibFailed );
  Transition ( switchFailed, calibRetryEv, encGroundCalib );
  Transition ( driveCalib, driveInvalidEv, prepareLeg );
  Transition ( driveCalib, driveValidEv, calibSuccess );
  Transition ( prepareLeg, legReadyEv, refineInterval );
  Transition ( prepareLeg, timeoutEv, calibFailed );
  Transition ( refineInterval, driveValidEv, calibSuccess );
  Transition ( refineInterval, timeoutEv, calibFailed );

  // the initial state
  initialize ( polarityTest );

  timeoutMark = 0;
  polarityMark = 0;
  driveMark = 0;

  polarityFlag = CALIB_POLARITY_DISABLE;
  encoderFlag = false;
  driveFlag = false;

  status = FAILURE;
  mode = GROUND;
  nextMode = UNSPECIFIED;   // Leave next mode unspecified 

  // Default ground offsets
  groundOffset = CALIB_DEF_GROUND_OFFSET;
  manualOffset = CALIB_DEF_MANUAL_OFFSET;
  switchOffset = CALIB_DEF_SWITCH_OFFSET;

  // Start by assuming that we are right side up
  upsideDownFlag = false;

  // Reset the module pointers to NULL
  enc = NULL;
  spd = NULL;
  stall = NULL;
  control = NULL;
} 

CalibMachine::~CalibMachine ( void ) {

  // deallocate events
  if ( polInvalidEv ) delete ( polInvalidEv );
  if ( polValidEv ) delete ( polValidEv );
  if ( stallEv ) delete ( stallEv );
  if ( encManualEv ) delete ( encManualEv );
  if ( encGroundEv ) delete ( encGroundEv );
  if ( encSwitchEv ) delete ( encSwitchEv );
  if ( encValidEv ) delete ( encValidEv );
  if ( fullRotEv ) delete ( fullRotEv );
  if ( switchEv ) delete ( switchEv );
  if ( noSwitchEv ) delete ( noSwitchEv );
  if ( noCalibRetryEv ) delete ( noCalibRetryEv );
  if ( calibRetryEv ) delete ( calibRetryEv );
  if ( driveInvalidEv ) delete ( driveInvalidEv );
  if ( driveValidEv ) delete ( driveValidEv );
  if ( legReadyEv ) delete ( legReadyEv );
  if ( timeoutEv ) delete ( timeoutEv );

  // deallocate states
  if ( polarityTest ) delete ( polarityTest );
  if ( polTestPos ) delete ( polTestPos );
  if ( encoderCalib ) delete ( encoderCalib );
  if ( polTestNeg ) delete ( polTestNeg );
  if ( calibFailed ) delete ( calibFailed );
  if ( encManualCalib ) delete ( encManualCalib );
  if ( encGroundCalib ) delete ( encGroundCalib );
  if ( encSwitchCalib ) delete ( encSwitchCalib );
  if ( driveCalib ) delete ( driveCalib );
  if ( clearSwitch ) delete ( clearSwitch );
  if ( seekSwitch ) delete ( seekSwitch );
  if ( switchFailed ) delete ( switchFailed );
  if ( prepareLeg ) delete ( prepareLeg );
  if ( calibSuccess ) delete ( calibSuccess );
  if ( refineInterval ) delete ( refineInterval );

}

void CalibMachine::reset( bool polarity, bool encoder, bool drive ) {

  if ( status != CALIBRATING ) {

    // Release the encoder module if it had remained grabbed from a 
    // previous successful calibration
    if ( status != FAILURE )
      MMReleaseModule ( enc, this );

    if ( polarity ) polarityFlag = CALIB_POLARITY_DISABLE;
    if ( encoder ) encoderFlag = false;
    if ( drive ) driveFlag = false;

    status = FAILURE;

  } else {
    MMWarning( "CalibMachine::reset", "Attempt to reset during calibration!" );
  }

}

void CalibMachine::reset( void ) {

  // Reset all calibration modes.
  reset( true, true, true );
}

void CalibMachine::setUpside( bool down ) {

  if ( status == FAILURE ) {

    // The encoders are not calibrated. So, we do not have to worry 
    // about resetting their calibration
    upsideDownFlag = down;

  } else if ( status == SUCCESS ) {

    // If there is a change, reset the encoder's current position 
    // to its current position + M_PI
    if ( upsideDownFlag != down ) {

      enc->reset( enc->getPosition() + M_PI );
      upsideDownFlag = down;

    }
  } else {
    MMWarning( "CalibMachine::reset", 
               "Attempt to set upside during calibration!" );
  }
}

void CalibMachine::init ( void ) {

  Floats offsets;

  hardware = MMGetHardware();

  if ( ( dcmotors = hardware->dcmotors ) == NULL )
    MMFatalError( "CalibMachine::CalibMachine",
                  "DC Motor hardware component is not supported!" );
  if ( ( analogIO = hardware->analogIO ) == NULL )
    MMFatalError( "CalibMachine::CalibMachine",
                  "Analog IO hardware component is not supported!" );
  if ( ( switches = hardware->switches ) == NULL )
    MMFatalError( "CalibMachine::CalibMachine",
                  "Switch hardware component is not supported!" );

  // Read the default calibration from the low level hardware
  dcmotors->getCalibration( getIndex(), & calib );

  // If the hardware layer does not provide polarity information, we
  // need to do it ourselves.
  if ( calib.drivePolarity == NO_POLARITY )
    polarityFlag = true;

  if ( ( enc = (EncoderReader *) 
         MMFindModule( ENCODERREADER_NAME, getIndex() )) == NULL)
    MMFatalError ( "CalibMachine::init", "Cannot find encoder reader!" );

  if ( ( spd = (SpeedFilter *) 
         MMFindModule( SPEEDFILTER_NAME, getIndex() )) == NULL)
    MMFatalError ( "CalibMachine::init", "Cannot find speed filter!" );

  if ( ( stall = (StallSensor *) 
         MMFindModule( STALLSENSOR_NAME, getIndex() )) == NULL)
    MMFatalError ( "CalibMachine::init", "Cannot find stall sensor!" );

  if ( ( control = (PositionControl *) 
         MMFindModule( POSITIONCONTROL_NAME, getIndex() )) == NULL)
    MMFatalError ( "CalibMachine::init", "Cannot find position controller!" );

  // Read the ground calibration offset data from the main
  // configuration symbol table
  offsets = MMGetArraySymbol( "ground_offset" );
  if ( getIndex() >= offsets.getCount() )
    groundOffset = CALIB_DEF_GROUND_OFFSET;
  else
    groundOffset = offsets.get( getIndex() );

  // Read the manual calibration offset data from the main
  // configuration symbol table
  offsets = MMGetArraySymbol( "manual_offset" );
  if ( getIndex() >= offsets.getCount() )
    manualOffset = CALIB_DEF_GROUND_OFFSET;
  else
    manualOffset = offsets.get( getIndex() );

  // Read the manual calibration offset data from the main
  // configuration symbol table
  offsets = MMGetArraySymbol( "switch_offset" );
  if ( getIndex() >= offsets.getCount() )
    switchOffset = CALIB_DEF_SWITCH_OFFSET;
  else
    switchOffset = offsets.get( getIndex() );

  // Read various configuration parameters from the symbol table
  calibTimeout = MMGetFloatSymbol( "calib_timeout", CALIB_TIMEOUT );
  seekCommand = MMGetFloatSymbol( "calib_command", CALIB_COMMAND );
  minPolSpeed = MMGetFloatSymbol( "calib_min_speed", CALIB_POL_MIN_SPEED );
  polTime = MMGetFloatSymbol( "calib_pol_time", CALIB_POL_TIME );
  stallTime = MMGetFloatSymbol( "calib_stall_time", CALIB_STALL_TIME );
  stallTolerance = MMGetFloatSymbol( "calib_stall_toler", 
                                     CALIB_STALL_TOLERANCE );
  calibRetry = bool( MMGetFloatSymbol( "calib_ground_retry", 1.0 ) );

  // Hardware switches corresponding to the home switches.
  // We assume that switches 8-13 are for leg home switches 0-5
  homeSwitch = 8 + getIndex();

  // Setup the stall detection parameters
  stall->setup( stallTolerance, polTime );

  StateMachine::init();

}

void CalibMachine::activate ( void ) {

  if ( nextMode == UNSPECIFIED ) {
    // If the mode we should use is not explicitly set, figure out
    // which calibration mode we are in based on DIP switch settings.

    if ( hardware->switches->read( CALIB_AUTO_SWITCH ) ) {
      if ( hardware->switches->read( CALIB_TYPE_SWITCH ) )
        mode = SWITCH;
      else
        mode = GROUND;

    } else
      mode = MANUAL;

  } else {
    // Calibration mode is explicitly set. Use the specified mode.
    mode = nextMode;
  }

  // Reset the next mode to UNSPECIFIED.
  nextMode = UNSPECIFIED;

  // The speed filter and the stall sensor are required by the calibration.
  MMGrabModule ( spd, this );
  MMGrabModule ( stall, this );

  // Stop motor rotation
  stopMotor();

  // Enable motor drives
  hardware->driveEnable( getIndex(), true );

  StateMachine::activate();
}

void CalibMachine::deactivate ( void ) {

  // Disable motor drives
  hardware->driveEnable( getIndex(), false );

  // Output the offset voltage to stop motor rotation
  analogIO->write( getIndex(), calib.driveOffset );

  // Release the previously grabbed modules
  MMReleaseModule( spd, this );
  MMReleaseModule( stall, this );

  // If the current state was prepareLegs, we need to release the
  // PositionControl module.
  if ( getCurState() == prepareLeg )
    MMReleaseModule( control, this );

  StateMachine::deactivate();
}

// =====================================================================
// CalibMachine private methods =====================================
// =====================================================================

// CalibMachine::stopMotor :
//
// Stops motor rotation.
//
void CalibMachine::stopMotor( void ) {

  analogIO->write( getIndex(), calib.driveOffset );
}

// CalibMachine::startMotor :
//
// Starts motor rotation with a given command.
//
void CalibMachine::startMotor( double command ) {

  analogIO->write( getIndex(), calib.driveOffset + command );
}

// CalibMachine::resetSpeed :
//
// Resets the speed measurement for polarity calibration purposes.
//
void CalibMachine::resetSpeed( void ) {
  // Reset the stall sensor for fresh measurement
  stall->reset();

  // Reset the latest speed measurement.
  lastSpeed = 0.0;

  // Record the current time for timing the speed measurement.
  polarityMark = MMReadTime();

}

// CalibMachine::measureSpeed :
//
// Monitors the speed and returns success when the speed remains above
// a certain threshold for a given amount of time.
//
bool CalibMachine::monitorSpeed( void ) {

  double speed = spd->getSpeed();
  double now = MMReadTime();

  if ( fabs( speed ) > minPolSpeed 
       && ( speed * lastSpeed > 0 ) ) {
    // If the speed is above the tolerance and is in the same
    // direction as the previous measurement, then keep waiting the
    // end of the minimum time/

    lastSpeed = speed;

    if ( now - polarityMark > polTime ) {
      // Rotation is satisfactorily large. Return success
      return true;
    }

  } else {
    // Oops, the motor either stopped or changed direction. Restart.

    lastSpeed = speed;
    polarityMark = now;
  }

  return false;
}

