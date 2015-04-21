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

/*
 *
 * RHEXLIB 
 *
 * Supervisor.cc
 *
 * This file and all of RHEXLIB are copyright 2000/2001 by the University
 * of Michigan. Copying and/or distributing require the expressed
 * written consent of the authors or maintainers of this code.
 *
 */

#include "Supervisor.hh"

#define OWNER ( ( Supervisor * ) owner )

#define RIGHT_RC_STICK 1
#define LEFT_RC_STICK 0

// Events ------------------------------------------------------------
bool Supervisor::CalCommand::check ( void ) { 

  // Left RC stick pushed forward
  return bool ( OWNER->rc->leftStick() == RemoteControl::NORTH );

}

bool Supervisor::CalFail::check ( void ) {

  int i;

  // return true if any motor reports failure
  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() == CalibMachine::CALIBRATING ) return false;
  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() == CalibMachine::FAILURE ) return true;

  return false;

}

bool Supervisor::CalSuccess::check ( void ) {

  // return true if all motors report success
  int i;

  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() != CalibMachine::SUCCESS ) return false;

  return true;

}

bool Supervisor::StandCommand::check ( void ) {

  // Left RC stick pushed back
  return bool ( OWNER->rc->leftStick() == RemoteControl::SOUTH );

}

bool Supervisor::StandDone::check ( void ) {

  return OWNER->standMach->isDone();

}

bool Supervisor::SitCommand::check ( void ) {

  // Left RC stick pushed left
  return bool ( OWNER->rc->leftStick() == RemoteControl::WEST );

}

bool Supervisor::SitDone::check ( void ) {

  return OWNER->sitMach->isDone();

}

bool Supervisor::PushupCommand::check ( void ) {

  // Left RC stick pushed forward
  return bool ( OWNER->rc->leftStick() == RemoteControl::NORTH );

}

bool Supervisor::StopCommand::check ( void ) {

  // Left RC stick pushed back
  return bool ( OWNER->rc->leftStick() == RemoteControl::SOUTH );

}

// States ------------------------------------------------------------
void Supervisor::Uncalibrated::entry ( void ) {}
void Supervisor::Uncalibrated::during ( void ) {}
void Supervisor::Uncalibrated::exit ( void ) {}

void Supervisor::Calibrating::entry ( void ) {

  int i;

  for ( i = 0; i < 6; i++ ) {

    OWNER->cm[i]->setMode( CalibMachine::MANUAL );
    MMGrabModule ( OWNER->cm[i], owner );

  }

}

void Supervisor::Calibrating::during ( void ) {}

void Supervisor::Calibrating::exit ( void ) {

  int i;

  for ( i = 0; i < 6; i++ )
    MMReleaseModule ( OWNER->cm[i], owner );

}

void Supervisor::Idle::entry ( void ) {}
void Supervisor::Idle::during ( void ) {}
void Supervisor::Idle::exit ( void ) {}

void Supervisor::RunningStand::entry ( void ) {

  int i;

  for ( i = 0; i < 6; i++ )  // enable motor drives
    OWNER->hw->driveEnable( i, true );
  
  MMGrabModule ( OWNER->standMach, owner );

}

void Supervisor::RunningStand::during ( void ) {}
void Supervisor::RunningStand::exit ( void ) {}

void Supervisor::Standing::entry ( void ) {}
void Supervisor::Standing::during ( void ) {}

void Supervisor::Standing::exit ( void ) {

  MMReleaseModule ( OWNER->standMach, owner );

}

void Supervisor::RunningSit::entry ( void ) {  

  MMGrabModule ( OWNER->sitMach, owner );

}

void Supervisor::RunningSit::during ( void ) {}
void Supervisor::RunningSit::exit ( void ) {

  int i;

  for ( i = 0; i < 6; i++ )  // disable motor drives
    OWNER->hw->driveEnable( i, false );

  MMReleaseModule ( OWNER->sitMach, owner );
  
}

void Supervisor::DoingPushups::entry ( void ) {

  MMGrabModule ( OWNER->puControl, owner );

}

void Supervisor::DoingPushups::during ( void ) {}

void Supervisor::DoingPushups::exit ( void ) {

  MMReleaseModule ( OWNER->puControl, owner );

}

Supervisor::Supervisor ( void ) : StateMachine ( "supervisor" ) {

  // allocate events
  calCommand = new CalCommand ( this, "CalCommand" );
  calFail = new CalFail ( this, "CalFail" );
  calSuccess = new CalSuccess ( this, "CalSuccess" );
  standCommand = new StandCommand ( this, "StandCommand" );
  standDone = new StandDone ( this, "StandDone" );
  sitCommand = new SitCommand ( this, "SitCommand" );
  sitDone = new SitDone ( this, "SitDone" );
  pushupCommand = new PushupCommand ( this, "PushupCommand" );
  stopCommand = new StopCommand ( this, "StopCommand" );

  // allocate states
  uncalibrated = new Uncalibrated ( this, "uncalibrated" );
  calibrating = new Calibrating ( this, "calibrating" );
  idle = new Idle ( this, "idle" );
  runningStand = new RunningStand ( this, "runningStand" );
  standing = new Standing ( this, "standing" );
  runningSit = new RunningSit ( this, "runningSit" );
  doingPushups = new DoingPushups ( this, "doingPushups" );

  // transitions ( in the form < From, Event, To > )
  Transition ( uncalibrated, calCommand, calibrating );
  Transition ( calibrating, calFail, uncalibrated );
  Transition ( calibrating, calSuccess, idle );
  Transition ( idle, standCommand, runningStand );
  Transition ( runningStand, standDone, standing );
  Transition ( standing, sitCommand, runningSit );
  Transition ( runningSit, sitDone, idle );
  Transition ( standing, pushupCommand, doingPushups );
  Transition ( doingPushups, stopCommand, runningStand );

  // the initial state
  initialize ( uncalibrated );

} 

Supervisor::~Supervisor ( void ) {

  if ( calCommand ) delete ( calCommand );
  if ( calFail ) delete ( calFail );
  if ( calSuccess ) delete ( calSuccess );
  if ( standCommand ) delete ( standCommand );
  if ( standDone ) delete ( standDone );
  if ( sitCommand ) delete ( sitCommand );
  if ( sitDone ) delete ( sitDone );
  if ( pushupCommand ) delete ( pushupCommand );
  if ( stopCommand ) delete ( stopCommand );
  if ( uncalibrated ) delete ( uncalibrated );
  if ( calibrating ) delete ( calibrating );
  if ( idle ) delete ( idle );
  if ( runningStand ) delete ( runningStand );
  if ( standing ) delete ( standing );
  if ( runningSit ) delete ( runningSit );
  if ( doingPushups ) delete ( doingPushups );

}

void Supervisor::init ( void ) {

  int i;

  // this is where to search for the needed modules and set up pointers to them

  if ( ( standMach = ( StandMachine * ) MMFindModule( STANDMACHINE_NAME, 0 )) == NULL)
     MMFatalError ( "Supervisor::init", "Cannot find Stand Machine" );
  
  if ( ( sitMach = ( SitMachine * ) MMFindModule( SITMACHINE_NAME, 0 )) == NULL)
     MMFatalError ( "Supervisor::init", "Cannot find Sit Machine" );
  
  if ( ( rc = ( RemoteControl * ) MMFindModule( REMOTECONTROL_NAME, 0 )) == NULL)
     MMFatalError ( "Supervisor::init", "Cannot find Remote Control" );

  if ( ( puControl = ( PushupController * ) MMFindModule( "pushupcontroller", 0 )) == NULL)
     MMFatalError ( "Supervisor::init", "Cannot find Push Up Controller" );

  for ( i = 0; i < 6; i++ )
    if ( ( cm[ i ] = ( CalibMachine * ) MMFindModule( CALIBMACHINE_NAME, i )) == NULL)
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

  if ( standMach->getState() == Module::ACTIVE ) MMReleaseModule ( standMach, this );
  if ( sitMach->getState() == Module::ACTIVE ) MMReleaseModule ( sitMach, this );
  if ( puControl->getState() == Module::ACTIVE ) MMReleaseModule ( puControl, this );

  for ( i=0; i<6; i++ )
    if ( cm[i]->getState() == Module::ACTIVE ) MMReleaseModule ( cm[i], this );

  StateMachine::deactivate();

}
