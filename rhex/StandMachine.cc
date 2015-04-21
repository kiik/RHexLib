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
 * $Id: StandMachine.cc,v 1.4 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/03/2001
 * Last Modified : Eric Klavins, 01/03/2001
 *
 ********************************************************************/

#include "ModuleManager.hh"
#include "StdModules.hh"
#include "StandMachine.hh"
#include "PositionControl.hh"

#define OWNER ( (StandMachine *) owner )

#define DEBUG_MSG( str ) // MMMessage( str );

// ------------------------------------------------------------------
// Events -----------------------------------------------------------

bool StandMachine::StandOneEv::check ( void ) { 

  return bool( MMReadTime() >= OWNER->mark + OWNER->timeOne );

}

bool StandMachine::StandTwoEv::check ( void ) { 

  return bool( MMReadTime() >= OWNER->mark + OWNER->timeTwo );

}

bool StandMachine::OffsetAdjEv::check ( void ) { 

  return bool( MMReadTime() >= OWNER->mark + OWNER->offsetAdjTime );

}

// ------------------------------------------------------------------ 
// States ------------------------------------------------------------

void StandMachine::StandOne::entry ( void ) { 
  int i;

  DEBUG_MSG( "StandMachine::StandOne::entry\n" );

  OWNER->mark = MMReadTime(); 

  for ( i = 0; i < 6; i++ ) {

    double pos = OWNER->enc[i]->getPosition();

    // Freeze the leg in place first. This is so that the PositionControl
    // module knows about the current state of the leg.
    OWNER->control[i]->freeze();

    // Command the leg to go to the appropriate angle
    if ( pos > 0.0 ) 

      OWNER->gotoAngle( i, OWNER->timeOne, 1.5 * M_PI );

    else 

      OWNER->gotoAngle( i, OWNER->timeOne, -0.5 * M_PI );
  }
}

void StandMachine::StandOne::during ( void ) {

  OWNER->trackCurrentProfile();

}

void StandMachine::StandTwo::entry ( void ) { 
  int i;

  DEBUG_MSG( "StandMachine::StandTwo::entry\n" );

  OWNER->mark = MMReadTime(); 

  for ( i = 0; i < 6; i++ ) {

    // Freeze the motor and go to the new target
    OWNER->control[i]->freeze();
    OWNER->gotoAngle( i, OWNER->timeTwo, OWNER->curLegOffset );
  }
}

void StandMachine::StandTwo::during ( void ) {

  OWNER->trackCurrentProfile();

}

void StandMachine::StandDone::entry ( void ) {

  int i;

  DEBUG_MSG( "StandMachine::StandDone::entry\n" );

  OWNER->done = true;

  OWNER->mark = MMReadTime(); 

  for ( i = 0; i < 6; i++ ) {

    // Go to the new leg offset
    OWNER->gotoAngle( i, 0.1, OWNER->curLegOffset );
  }
}

void StandMachine::StandDone::during ( void ) {

  OWNER->trackCurrentProfile();

}

// ------------------------------------------------------------------
// StandMachine Methods ---------------------------------------------

StandMachine::StandMachine ( void ) : StateMachine( STANDMACHINE_NAME ) {

  // allocate events
  standOneEv = new StandOneEv ( this, "StandOneEv" );
  standTwoEv = new StandTwoEv ( this, "StandTwoEv" );
  offsetAdjEv = new OffsetAdjEv ( this, "OffsetAdjEv" );

  // allocate states
  standOne = new StandOne ( this, "StandOne" );
  standTwo = new StandTwo ( this, "StandTwo" );
  standDone = new StandDone ( this, "StandDone" );

  // add transitions ( this defines the structure of the machine )
  // 
  //           From State      Event         To State
  //------------------------------------------------------------
  Transition ( standOne,       standOneEv,   standTwo );
  Transition ( standTwo,       standTwoEv,   standDone );
  Transition ( standDone,      offsetAdjEv,  standDone );

  // define state machine
  initialize ( standOne );

}

StandMachine::~StandMachine () {

  if ( standOneEv ) delete ( standOneEv );
  if ( standTwoEv ) delete ( standTwoEv );
  if ( offsetAdjEv ) delete ( offsetAdjEv );

  if ( standOne ) delete ( standOne );
  if ( standTwo ) delete ( standTwo );
  if ( standDone ) delete ( standDone );

}

void StandMachine::init ( void ) {

  Floats temp;
  int i;

  for ( i = 0; i < 6; i++ )
    if ( ( control[i] = ( PositionControl * ) 
           MMFindModule( POSITIONCONTROL_NAME, i )) == NULL)
      MMFatalError ( "StandMachine::init", "Cannot find motor controller!" );

  for ( i = 0; i < 6; i++ )
    if ( ( enc[i] = ( EncoderReader * ) 
           MMFindModule( ENCODERREADER_NAME, i )) == NULL)
      MMFatalError ( "StandMachine::init", "Cannot find encoder reader!" );

  // set up configuration parameters
  timeOne = MMGetFloatSymbol( "stand_one_time", STAND_ONETIME_DFLT );
  timeTwo = MMGetFloatSymbol( "stand_two_time", STAND_TWOTIME_DFLT );
  offsetAdjTime = MMGetFloatSymbol( "stand_adj_time", STAND_ADJTIME_DFLT );

  curLegOffset = STAND_LEGOFFSET_DFLT;

  temp = MMGetArraySymbol( "stand_kp" );          // P gain
  for ( i = 0; i < temp.getCount(); i++ ) 
    gains[i].kp = temp.get(i);

  for ( i = temp.getCount(); i < 6; i++ ) {
    gains[i].kp = STAND_KP_DFLT;
	//    MMWarning( "StandMachine::init", 
	//			   "Using default parameters for stand_kp!" );
  }

  temp = MMGetArraySymbol( "stand_kd" );          // D gain
  for ( i = 0; i < temp.getCount(); i++ ) 
    gains[i].kd = temp.get(i);

  for ( i = temp.getCount(); i < 6; i++ ) {
    gains[i].kd = STAND_KD_DFLT;
	//    MMWarning( "StandMachine::init", 
	//               "Using default parameters for stand_kd!" );
  }

  StateMachine::init();

}

void StandMachine::activate ( void ) {

  int i;

  // Grab the position controllers and the encoder reader
  for ( i = 0; i < 6; i++ ) {

    MMGrabModule ( control[i], this );
    // Save the old PD gains and set the new ones
    control[i]->getGains( &oldGains[i] );
    control[i]->setGains( &gains[i] );

    MMGrabModule ( enc[i], this );
  }
 
  done = false;

  StateMachine::activate();

}

void StandMachine::deactivate ( void ) {

  int i;

  // Release the modules grabbed during activation
  for ( i = 0; i < 6; i++ ) {

    control[i]->setGains( &oldGains[i] );

    MMReleaseModule ( control[i], this );
    MMReleaseModule ( enc[i], this );
  }
}

// StandMachine::gotoAngle : A utility to set an angle target
void StandMachine::gotoAngle( uint leg, double time, float angle ) {

  MotorTarget_t curTarget;
  double now = MMReadTime();

  control[leg]->getTarget( &curTarget );

  prof[leg].setup ( now, now + time, curTarget.pos, angle );
}

// StandMachine::trackCurrentProfile : Tracks the currently setup leg
// profile.  Must be called by the during() function of the states for
// setting motor targets
void StandMachine::trackCurrentProfile ( void ) {

  int i;
  MotorTarget_t tar;

  for ( i = 0; i < 6; i++ ) {

    tar = prof[i].getVal();

    control[i]->setTarget( &tar );
  }
}
