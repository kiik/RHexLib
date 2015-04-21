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
 * $Id: StartupMachine.cc,v 1.3 2001/08/14 02:35:54 ulucs Exp $
 *
 * Created       : Uluc Saranli, 01/22/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// Local includes
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "StartupMachine.hh"
#include "CalibMachine.hh"

#define OWNER ( (StartupMachine *) owner )

#define DEBUG_MSG( str ) // MMMessage( str );

// Did calibration succeed? return true if all motors report success
bool StartupMachine::CalibSuccess::check ( void ) { 

  int i;

  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() != CalibMachine::SUCCESS ) return false;
  
  return true;

}

// Did calibration fail? return true if there is a motor that failed
bool StartupMachine::CalibFail::check ( void ) { 
  int i;

  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() == CalibMachine::CALIBRATING ) return false;

  for ( i = 0; i < 6; i++ )
    if ( OWNER->cm[i]->getStatus() == CalibMachine::FAILURE ) return true;
  
  return false;
}

// Is standing done?
bool StartupMachine::StandDoneEv::check ( void ) {

  return OWNER->sm->isDone();
}

// -------------------------------------------------------------------
// States ------------------------------------------------------------

//***************
// Calibrating
//***************
void StartupMachine::Calibrating::entry ( void ) {

  int i;

  DEBUG_MSG("StartupMachine::Calibrating::entry\n");

  for ( i = 0; i < 6; i++ ) {

    OWNER->cm[i]->setMode( OWNER->calibMode );
    MMGrabModule ( OWNER->cm[i], owner );
  }

  OWNER->status = CALIBRATING;
}

void StartupMachine::Calibrating::exit ( void ) {

  int i;

  DEBUG_MSG("StartupMachine::Calibrating::exit\n");

  for ( i = 0; i < 6; i++ )
    MMReleaseModule ( OWNER->cm[i], owner );

}

//***************
// Standing
//***************
void StartupMachine::Standing::entry ( void ) {

  int i;

  DEBUG_MSG("StartupMachine::Standing::entry\n");

  for ( i = 0; i < 6; i++ )
    OWNER->hw->driveEnable( i, true );  // enable motor drives

  MMGrabModule ( OWNER->sm, owner );

  OWNER->status = STANDING;
}

void StartupMachine::Standing::exit ( void ) {

  DEBUG_MSG("StartupMachine::Standing::exit\n");

}

//***************
// DoneStanding
//**************
void StartupMachine::DoneStanding::entry   ( void ) {

  DEBUG_MSG("StartupMachine::DoneStanding::entry\n");

  OWNER->status = SUCCESS;
}

//***************
// FailedState
//***************
void StartupMachine::FailedState::entry   ( void ) {

  DEBUG_MSG("StartupMachine::FailedState::entry\n");

  OWNER->status = FAILURE;
}

// StandMachine Methods --------------------------------------------------

StartupMachine::StartupMachine( void ) : StateMachine( STARTUPMACHINE_NAME ) {

  // allocate events
  calibSuccess = new CalibSuccess ( this );
  calibFail = new CalibFail ( this );
  standDoneEv = new StandDoneEv ( this );

  // allocate states
  failedState = new FailedState ( this );
  calibrating = new Calibrating ( this );
  standing  = new Standing ( this );
  doneStanding  = new DoneStanding ( this );
  

  // transitions ( this defines the structure of the machine )
  // 
  //           From State        Event            To State
  //------------------------------------------------------------
  Transition ( calibrating,      calibSuccess,    standing );
  Transition ( calibrating,      calibFail,       failedState );
  Transition ( standing,         standDoneEv,     doneStanding );

  // define state machine
  initialize ( calibrating );

  //  calibMode = CalibMachine::GROUND;

  status = FAILURE;
}

StartupMachine::~StartupMachine () {

  if ( calibSuccess ) delete ( calibSuccess );
  if ( calibFail ) delete ( calibFail );
  if ( standDoneEv ) delete ( standDoneEv );

  if ( failedState ) delete ( failedState );
  if ( calibrating ) delete ( calibrating );
  if ( standing  ) delete ( standing );
  if ( doneStanding  ) delete ( doneStanding );
}

void StartupMachine::init ( void ) {

  // this is where to search for the needed modules and set up pointers to them

  if ( ( sm = ( StandMachine * ) 
         MMFindModule( STANDMACHINE_NAME, 0 )) == NULL )
     MMFatalError ( "StartupMachine::init", "Cannot find Stand Machine" );
  
  int i;

  for ( i = 0; i < 6; i++ )
    if ( ( cm[i] = ( CalibMachine * ) 
           MMFindModule( CALIBMACHINE_NAME, i ) ) == NULL )
       MMFatalError ( "StartupMachine::init", "Cannot find Calibration Machine" );

  hw = MMGetHardware();

  StateMachine::init();

}

void StartupMachine::deactivate ( void ) {

  int i;

  if ( status == CALIBRATING ) {
    // Deactivate the calibration machines if we got killed prematurely.
	for ( i = 0; i < 6; i++ )
	  MMReleaseModule ( cm[i], this );

  } else if ( status == STANDING || status == SUCCESS )
    // Deactivate the standing machine if it was active
    MMReleaseModule ( sm, this );

  StateMachine::deactivate();
}
