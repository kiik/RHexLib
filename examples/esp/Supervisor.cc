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
 * $Id: Supervisor.cc,v 1.1 2001/08/06 17:01:44 ulucs Exp $
 *
 ********************************************************************/

#include "Supervisor.hh"

#define OWNER ( ( Supervisor * ) owner )

// Events ------------------------------------------------------------
bool Supervisor::StartCommand::check ( void ) {

  return bool ( OWNER->rc->leftStick() == RemoteControl::NORTH );

}

bool Supervisor::CalFail::check ( void ) {

  int i;

  // return true if any motor reports failure
  for ( i = 0; i < 6; i++ )
    if ( OWNER->calibMachine[i]->getStatus() == CalibMachine::FAILURE )
      return true;

  return false;

}

bool Supervisor::CalSuccess::check ( void ) {

  // return true if all motors report success
  int i;

  //  return true; // until I figure out why this doesn't work

  for ( i = 0; i < 6; i++ )
    if ( OWNER->calibMachine[i]->getStatus() != CalibMachine::SUCCESS )
      return false;

  return true;

}

bool Supervisor::DoneStanding::check ( void ) {

  return OWNER->standMachine->isDone();

}

bool Supervisor::AccWalkCommand::check ( void ) {

  return bool ( OWNER->rc->leftStick() == RemoteControl::WEST
           && ( OWNER->rc->rightStick() == RemoteControl::NORTH
             || OWNER->rc->rightStick() == RemoteControl::SOUTH ) );

}

bool Supervisor::WalkCommand::check ( void ) {

  return bool ( OWNER->rc->leftStick() == RemoteControl::CENTER
           && ( OWNER->rc->rightStick() == RemoteControl::NORTH
             || OWNER->rc->rightStick() == RemoteControl::SOUTH ) );

}

bool Supervisor::UpToSpeed::check ( void ) {

  return bool ( MMReadTime() >= OWNER->mark + ACCEL_TIME );

}

bool Supervisor::NoCommand::check ( void ) {

  return bool ( OWNER->rc->rightStick() == RemoteControl::CENTER
             && OWNER->rc->leftStick() != RemoteControl::WEST);

}

bool Supervisor::StopCommand::check ( void ) {

  return bool ( OWNER->rc->rightStick() == RemoteControl::CENTER
             && OWNER->rc->leftStick() == RemoteControl::WEST );

}

bool Supervisor::DoneDecel::check ( void ) {

  return bool ( MMReadTime() >= OWNER->mark + ACCEL_TIME );

}

// States ------------------------------------------------------------
void Supervisor::UnCalibrated::entry ( void ) {

  MMMessage ( "Entering Supervisor::UnCalibrated\n" );

}

void Supervisor::UnCalibrated::during ( void ) {}
void Supervisor::UnCalibrated::exit ( void ) {}

void Supervisor::Calibrating::entry ( void ) {

  MMMessage ( "Entering Supervisor::Calibrating\n" );

  int i;

  for ( i = 0; i < 6; i++ ) {

    OWNER->calibMachine[i]->setMode( CalibMachine::MANUAL );
    MMGrabModule ( OWNER->calibMachine[i], owner );

  }

}

void Supervisor::Calibrating::during ( void ) {}

void Supervisor::Calibrating::exit ( void ) {

  int i;

  for ( i = 0; i < 6; i++ )
    MMReleaseModule ( OWNER->calibMachine[i], owner );

}

void Supervisor::Standing::entry ( void ) {

  MMMessage ( "Entering Supervisor::Standing\n" );

  int i;

  for ( i = 0; i < 6; i++ )  // enable motor drives
    OWNER->hw->driveEnable( i, true );

  MMGrabModule ( OWNER->standMachine, owner );

}

void Supervisor::Standing::during ( void ) {}
void Supervisor::Standing::exit ( void ) {}

void Supervisor::Ready::entry ( void ) {

  MMMessage ( "Entering Supervisor::Ready\n" );

  if ( OWNER->walkMachine->getState() == Module::ACTIVE )
    OWNER->walkMachine->setForwardCommand ( 0.0 );

}

void Supervisor::Ready::during ( void ) {}

void Supervisor::Ready::exit ( void ) {

  if ( OWNER->standMachine->getState() == Module::ACTIVE )
    MMReleaseModule ( OWNER->standMachine, owner );

}

void Supervisor::Accelerating::entry ( void ) {

  MMMessage ( "Entering Supervisor::Accelerating\n" );

  // set up direction
  OWNER->setDirection();

  // set up an acceleration profile
  OWNER->mark = MMReadTime();
  OWNER->accelProfile.setupLinear( OWNER->mark, 0.25, 
                                   OWNER->mark + ACCEL_TIME, 1.0 );
  OWNER->walkMachine->setForwardCommand ( 
    OWNER->accelProfile.getVal( MMReadTime() ) * OWNER->direction 
  );
  OWNER->walkMachine->setTurnCommand ( 0.0 ); // no turning in this example
  MMGrabModule ( OWNER->walkMachine, owner );

}

void Supervisor::Accelerating::during ( void ) {

  OWNER->walkMachine->setForwardCommand ( 
    OWNER->accelProfile.getVal ( MMReadTime() ) * OWNER->direction 
  );

}

void Supervisor::Accelerating::exit ( void ) {}

void Supervisor::Walking::entry ( void ) {

  MMMessage ( "Entering Supervisor::Walking\n" );

  if ( OWNER->walkMachine->getState() == Module::INACTIVE ) {
 
    OWNER->setDirection();
    OWNER->mark = MMReadTime();
    // a dummy profile, returns constant 1.0
    OWNER->accelProfile.setupLinear ( 0.0, 1.0, 1.0, 1.0 );
    OWNER->walkMachine->setTurnCommand ( 0.0 ); // no turning in this example
    MMGrabModule ( OWNER->walkMachine, owner );

  }

}
void Supervisor::Walking::during ( void ) {

  OWNER->walkMachine->setForwardCommand ( 
    OWNER->accelProfile.getVal ( MMReadTime() ) * OWNER->direction 
  );

}

void Supervisor::Walking::exit ( void ) {}

void Supervisor::Decelerating::entry ( void ) {

  MMMessage ( "Entering Supervisor::Decelerating\n" );

  OWNER->mark = MMReadTime();
  OWNER->accelProfile.setupLinear ( OWNER->mark, 1.0, 
                                    OWNER->mark + ACCEL_TIME, 0.25 );

}

void Supervisor::Decelerating::during ( void ) {

  OWNER->walkMachine->setForwardCommand ( 
    OWNER->accelProfile.getVal ( MMReadTime() ) * OWNER->direction 
  );

}

void Supervisor::Decelerating::exit ( void ) {}

void Supervisor::setDirection ( void ) {

  if ( rc->rightStick() == RemoteControl::NORTH )
    direction = 1;
  else direction = -1;

}

Supervisor::Supervisor ( void ) : StateMachine ( "supervisor" ) {

  // allocate events
  startCommand = new StartCommand ( this, "StartCommand" );
  calFail = new CalFail ( this, "CalFail" );
  calSuccess = new CalSuccess ( this, "CalSuccess" );
  doneStanding = new DoneStanding ( this, "DoneStanding" );
  accWalkCommand = new AccWalkCommand ( this, "AccWalkCommand" );
  walkCommand = new WalkCommand ( this, "WalkCommand" );
  upToSpeed = new UpToSpeed ( this, "UpToSpeed" );
  noCommand = new NoCommand ( this, "NoCommand" );
  stopCommand = new StopCommand ( this, "StopCommand" );
  doneDecel = new DoneDecel ( this, "DoneDecel" );

  // allocate states
  unCalibrated = new UnCalibrated ( this, "unCalibrated" );
  calibrating = new Calibrating ( this, "calibrating" );
  standing = new Standing ( this, "standing" );
  ready = new Ready ( this, "ready" );
  accelerating = new Accelerating ( this, "accelerating" );
  walking = new Walking ( this, "walking" );
  decelerating = new Decelerating ( this, "decelerating" );

  // transitions ( in the form < From, Event, To > )
  Transition ( unCalibrated, startCommand, calibrating );
  Transition ( calibrating, calFail, unCalibrated );
  Transition ( calibrating, calSuccess, standing );
  Transition ( standing, doneStanding, ready );
  Transition ( ready, accWalkCommand, accelerating );
  Transition ( ready, walkCommand, walking );
  Transition ( accelerating, upToSpeed, walking );
  Transition ( accelerating, noCommand, ready );
  Transition ( walking, noCommand, ready );
  Transition ( walking, stopCommand, decelerating );
  Transition ( decelerating, doneDecel, ready );

  // the initial state
  initialize ( unCalibrated );

} 

Supervisor::~Supervisor ( void ) {

  if ( startCommand ) delete ( startCommand );
  if ( calFail ) delete ( calFail );
  if ( calSuccess ) delete ( calSuccess );
  if ( doneStanding ) delete ( doneStanding );
  if ( accWalkCommand ) delete ( accWalkCommand );
  if ( walkCommand ) delete ( walkCommand );
  if ( upToSpeed ) delete ( upToSpeed );
  if ( noCommand ) delete ( noCommand );
  if ( stopCommand ) delete ( stopCommand );
  if ( doneDecel ) delete ( doneDecel );
  if ( unCalibrated ) delete ( unCalibrated );
  if ( calibrating ) delete ( calibrating );
  if ( standing ) delete ( standing );
  if ( ready ) delete ( ready );
  if ( accelerating ) delete ( accelerating );
  if ( walking ) delete ( walking );
  if ( decelerating ) delete ( decelerating );

}

void Supervisor::init ( void ) {

  int i;

  // this is where to search for the needed modules and set up pointers to them

  if ( ( standMachine = ( StandMachine * ) 
         MMFindModule( STANDMACHINE_NAME, 0 )) == NULL)
    MMFatalError ( "Supervisor::init", "Cannot find Stand Machine" );
  
  if ( ( rc = ( RemoteControl * ) 
         MMFindModule( REMOTECONTROL_NAME, 0 )) == NULL)
    MMFatalError ( "Supervisor::init", "Cannot find Remote Control" );

  if ( ( walkMachine = ( RHexWalker * ) 
         MMFindModule( RHEXWALKER_NAME, 0 )) == NULL)
    MMFatalError ( "Supervisor::init", "Cannot find Push Up Controller" );

  for ( i = 0; i < 6; i++ )
    if ( ( calibMachine[ i ] = ( CalibMachine * ) 
           MMFindModule( CALIBMACHINE_NAME, i )) == NULL)
      MMFatalError ( "Supervisor::init", "Cannot find Calibration Machine" );

  hw = MMGetHardware(); 

  StateMachine::init();

}

void Supervisor::activate ( void ) {

  // Configure the RC joysticks
  rc->configure ( 0.005, 0.3 );
  rc->setThreshold ( 0.5 );

  // Grab and activate the remote control interface
  MMGrabModule( rc, this );

  StateMachine::activate();

}

void Supervisor::deactivate ( void ) {

  int i;

  // Release the previously grabbed modules
  MMReleaseModule( rc, this );

  if ( standMachine->getState() == Module::ACTIVE ) 
    MMReleaseModule ( standMachine, this );
  if ( walkMachine->getState() == Module::ACTIVE )
    MMReleaseModule ( walkMachine, this );

  for ( i=0; i<6; i++ )
    if ( calibMachine[i]->getState() == Module::ACTIVE )
      MMReleaseModule ( calibMachine[i], this );

  StateMachine::deactivate();

}
