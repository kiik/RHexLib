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
 * $Id: WalkMachine.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include "ModuleManager.hh"
#include "StdModules.hh"
#include "WalkMachine.hh"
#include "PositionControl.hh"
#include "basicmath.hh"

#define OWNER ( ( WalkMachine * ) owner )

#define DEBUG_MSG( str ) // MMMessage( str );

// Parameters type class definitions ---------------------------------
WalkParam_t::WalkParam_t( WalkParam_t &old ) {

  this->operator=( old );

}

WalkParam_t& WalkParam_t::operator=( const WalkParam_t &old ) {
  int i;

  tripodTime = old.tripodTime;
  standAdjTime = old.standAdjTime;
  cpgPeriod = old.cpgPeriod;

  for ( i = 0; i < 6; i++ ) {

    dutyFactor[i] = old.dutyFactor[i];
    sweepAngle[i] = old.sweepAngle[i];
    legOffset[i] = old.legOffset[i];
    gains[i].kp = old.gains[i].kp;
    gains[i].kd = old.gains[i].kd;
  }

  smooth = old.smooth;
  turnOffset = old.turnOffset;
  turnSweepAngle = old.turnSweepAngle;
  turnDutyFactor = old.turnDutyFactor;
  cubic = old.cubic;

  return *this;
}


// Events ------------------------------------------------------------

// check whether the time to put up one tripod is up and return true if it is
bool WalkMachine::TripodReady::check ( void ) {

  return bool( MMReadTime() >= OWNER->mark + OWNER->curParams.tripodTime );

}

// check whether the first half of a cpg cycle is done with
bool WalkMachine::HalfDone::check ( void ) { 

  return bool ( OWNER->cpg->getVal() >= 0.5 );

}

// whether the second half of a cpg cycle is done with
bool WalkMachine::AllDone::check ( void ) { 

  return bool ( OWNER->cpg->getVal() >= 1.0 || OWNER->cpg->getVal() < 0.25 ); 

}

// checks whether there is no command at the beginning or halfway
// through a cycle
bool WalkMachine::NoCommand::check ( void ) { 

  // check that we are at cpg = 0 or 1/2 and that there is not RC command.

  return 
    bool( ( fabs( OWNER->cpg->getVal() - 0.510 ) < 0.010
            || OWNER->cpg->getVal() >= 1.0
            || OWNER->cpg->getVal() < 0.10 )
          && ( fabs( OWNER->speedCommand ) < 0.1 )// no forw/backw command
          && ( fabs( OWNER->turnCommand ) < 0.1 ) // no turn command
          );

}

// checks whether there is a command from the user
bool WalkMachine::WalkTrigEv::check ( void ) { 

  return 
    bool( ( fabs( OWNER->speedCommand ) > 0.1 )  // forw/backw command
          || ( fabs( OWNER->turnCommand ) > 0.1 ) ); // turn command

}

// check whether the time to put the tripods down is up and return true if it is
bool WalkMachine::TripodDown::check ( void ) {

  return bool( MMReadTime() >= OWNER->mark + OWNER->curParams.tripodTime );

}

// check whether the time to put the tripods down is up and return true if it is
bool WalkMachine::StandAdjEv::check ( void ) {

  return bool( MMReadTime() >= OWNER->mark + OWNER->curParams.standAdjTime );

}

// States -------------------------------------------------------------

//****************
// WaitingCommand
//****************
void WalkMachine::WaitingCommand::entry ( void ) {

  int i;

  DEBUG_MSG( "WalkMachine::WaitingCommand::entry\n" );

  OWNER->done = true;

  OWNER->mark = MMReadTime(); 

  // Force the parameter settings to take effect
  OWNER->useNextParams();

  for ( i = 0; i < 6; i++ ) {

    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );
    OWNER->gotoAngle( i, 0.1, OWNER->curParams.legOffset[i], 0 );
  }
}

void WalkMachine::WaitingCommand::during ( void ) {

  OWNER->trackCurrentProfile();

}

void WalkMachine::WaitingCommand::exit ( void ) {

  DEBUG_MSG( "WalkMachine::WaitingCommand::exit\n" );

  // We are starting to walk, so turn off the done flag
  OWNER->done = false;
}

//****************
// LiftingTripod
//****************

// set up the profiles to raise one of the tripods
void WalkMachine::LiftingTripod::entry ( void ) {

  int i;
  int direction;

  DEBUG_MSG( "WalkMachine::LiftingTripod::entry\n" );

  OWNER->mark = MMReadTime(); 

  // Force the parameter settings to take effect
  OWNER->useNextParams();

  // Determine the direction to lift the tripod
  if ( OWNER->speedCommand > 0.25 )
    direction = -1;
  else
    direction = 1;
  
  // Set the tripods to go to their appropriate starting location
  for ( i = 0; i < 6; i++ ) {

    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );
    OWNER->control[i]->freeze();

    if ( ( i % 2 ) == 0 )
      OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                        OWNER->curParams.legOffset[i] + M_PI, direction );
    else
      OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                        OWNER->curParams.legOffset[i], 0 );
  }

}

void WalkMachine::LiftingTripod::during ( void ) {

  OWNER->trackCurrentProfile();

}

// After this exits, we go into walking. So we have to grab the CPG
// and reset it. Then we have to set the parameters of the
// CPGProfilers
void WalkMachine::LiftingTripod::exit ( void ) {

  int i;

  DEBUG_MSG( "WalkMachine::LiftingTripod::exit\n" );

  // grab the CPG and set up the CPGProfiler parameters
  MMGrabModule ( OWNER->cpg, OWNER );

  // Force the parameter settings to take effect
  OWNER->useNextParams();

  OWNER->cpg->reset( OWNER->curParams.cpgPeriod, 0 );

  OWNER->cpg->setPeriod( OWNER->curParams.cpgPeriod );

  for ( i = 0; i < 6; i++ ) {
    // Update the motor gains
    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );

    OWNER->cpgProf[i].initParams( OWNER->curParams.dutyFactor[ i ], 
                                  OWNER->curParams.sweepAngle[ i ], 
                                  OWNER->curParams.legOffset[ i ], 
                                  OWNER->curParams.smooth );
    OWNER->cpgProf[i].setCubic( OWNER->curParams.cubic );
  }
}

//****************
// FirstHalf
//****************

// At the entry of the first half cycle, we have to mark the positions
// of the legs so that the CPG profiler is continuous between offset
// changes. Also, here is where to change the offset because of
// turning commands, etc.
void WalkMachine::FirstHalf::entry ( void ) {

  int i;

  DEBUG_MSG( "WalkMachine::FirstHalf::entry\n" );

  OWNER->firstHalfFlag = true;

  // Force the parameter settings to take effect
  OWNER->useNextParams();

  // Update the motor gains
  for ( i = 0; i < 6; i++ )
    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );

  // Setup the profiles for the first half
  OWNER->setupProfiles ( );

}

void WalkMachine::FirstHalf::during ( void ) {

  OWNER->trackCPGProfile();
 
}

//****************
// SecondHalf
//****************

// similar to WalkMachine::FirstHalf::entry
void WalkMachine::SecondHalf::entry ( void ) {

  int i;

  DEBUG_MSG( "WalkMachine::SecondHalf::entry\n" );

  OWNER->firstHalfFlag = false;

  // Force the parameter settings to take effect
  OWNER->useNextParams();

  // Update the motor gains
  for ( i = 0; i < 6; i++ )
    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );

  // Setup the profiles for the second half
  OWNER->setupProfiles ( );

}

void WalkMachine::SecondHalf::during ( void ) { 

  OWNER->trackCPGProfile();
 
}

//****************
// LoweringTripod
//****************

// set up the profiles to lower all the leggs to standing position
void WalkMachine::LoweringTripod::entry ( void ) {

  int i;
  int direction;

  DEBUG_MSG( "WalkMachine::LoweringTripod::entry\n" );

  OWNER->mark = MMReadTime(); 

  // Determine the direction to lift the tripod
  if ( OWNER->lastSpeedCommand > 0.25 )
    direction = -1;
  else
    direction = 1;
  
  // Force the parameter settings to take effect
  OWNER->useNextParams();

  // Command the tripods to come down
  for ( i = 0; i < 6; i++ ) {

    OWNER->control[i]->setGains( &(OWNER->curParams.gains[i]) );
    OWNER->control[i]->freeze();

    if ( OWNER->firstHalfFlag ) {
      // If the first tripod(left) was up, lower legs 0, 2 and 4 in the 
      // direction of the locomotion

      if ( ( i % 2 ) == 0 )
        OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                          OWNER->curParams.legOffset[i], direction );
      else
        OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                          OWNER->curParams.legOffset[i], 0 );

    } else {
      // If the second tripod(right) was up, lower legs 0, 2 and 4 in the
      // cirection of the locomotion

      if ( ( i % 2 ) == 0 )
        OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                          OWNER->curParams.legOffset[i], 0 );
      else
        OWNER->gotoAngle( i, OWNER->curParams.tripodTime, 
                          OWNER->curParams.legOffset[i], direction );
    }
  }

  // Release the CPG module
  MMReleaseModule ( OWNER->cpg, OWNER );

}

void WalkMachine::LoweringTripod::during ( void ) {

  OWNER->trackCurrentProfile();

}

// StandMachine Methods --------------------------------------------------

WalkMachine::WalkMachine ( void ) : StateMachine( WALKMACHINE_NAME ) {

  int i;

  // allocate events
  tripodReady = new TripodReady ( this );
  halfDone = new HalfDone ( this );
  allDone = new AllDone ( this );
  noCommand = new NoCommand ( this );
  walkTrigEv = new WalkTrigEv ( this );
  tripodDown = new TripodDown ( this );
  standAdjEv = new StandAdjEv ( this );

  // allocate states
  waitingCommand = new WaitingCommand ( this );
  liftingTripod = new LiftingTripod ( this );
  firstHalf = new FirstHalf ( this );
  secondHalf = new SecondHalf ( this );
  loweringTripod = new LoweringTripod ( this ); 

  // add transitions ( this defines the structure of the machine )
  // 
  //           From State          Event          To State
  //------------------------------------------------------------
  Transition ( waitingCommand,     walkTrigEv,   liftingTripod );
  Transition ( waitingCommand,     standAdjEv,    waitingCommand );

  Transition ( liftingTripod,      tripodReady,   firstHalf );
  Transition ( firstHalf,          halfDone,      secondHalf );
  Transition ( secondHalf,         allDone,       firstHalf );

  Transition ( firstHalf,          noCommand,     loweringTripod );
  Transition ( secondHalf,         noCommand,     loweringTripod );

  Transition ( loweringTripod,     tripodDown,    waitingCommand );

  // Choose the initial state
  initialize ( waitingCommand );
  
  // Start out with the neutral command set
  speedCommand = turnCommand = 0.0;
  lastSpeedCommand = lastTurnCommand = 0.0;

  // Start by assuming that we are right side up
  upsideDownFlag = false;
  firstHalfFlag = true;

  // these are the parameter defaults
  curParams.tripodTime = WALK_TRIPODTIME_DFLT;
  curParams.standAdjTime = WALK_ADJTIME_DFLT;
  curParams.cpgPeriod =  WALK_CPGPERIOD_DFLT;

  for ( i = 0; i < 6; i++ ) {

    curParams.dutyFactor[i] =  WALK_DUTYFACTOR_DFLT;
    curParams.sweepAngle[i] =  WALK_SWEEPANGLE_DFLT;
    curParams.legOffset[i]  =  WALK_LEGOFFSET_DFLT;
    curParams.gains[i].kp = WALK_KP_DFLT;
    curParams.gains[i].kd = WALK_KD_DFLT;
  }

  curParams.smooth  =  WALK_SMOOTH_DFLT;
  curParams.turnOffset = WALK_TURNOFFSET_DFLT;
  curParams.turnSweepAngle = WALK_TURNSWEEPANGLE_DFLT;
  curParams.turnDutyFactor = WALK_TURNDUTYFACTOR_DFLT;
  curParams.cubic = WALK_CUBIC_DFLT;

  nextParams = NULL;

  // Create the CPG module to be used in the generation of the leg profiles
  cpg = new CPG( "walkcpg", 0 );
}

WalkMachine::~WalkMachine () {

  // deallocate events
  if ( tripodReady ) delete ( tripodReady );
  if ( halfDone ) delete ( halfDone );
  if ( allDone ) delete ( allDone );
  if ( noCommand ) delete ( noCommand );
  if ( walkTrigEv ) delete ( walkTrigEv );
  if ( tripodDown ) delete ( tripodDown );
  if ( standAdjEv ) delete ( standAdjEv );

  // deallocate states
  if ( waitingCommand ) delete ( waitingCommand );
  if ( liftingTripod ) delete ( liftingTripod );
  if ( firstHalf ) delete ( firstHalf );
  if ( secondHalf ) delete ( secondHalf );
  if ( loweringTripod ) delete ( loweringTripod ); 

  if ( cpg ) delete ( cpg );
}

void WalkMachine::init ( void ) {

  int i;

  // Acquire pointers to modules that this machine will need to use.
  for ( i = 0; i < 6; i++ )
    if ( ( control[i] = ( PositionControl * ) 
           MMFindModule( POSITIONCONTROL_NAME, i )) == NULL)
      MMFatalError ( "StandMachine::init", "Cannot find motor controller!" );

  // Add the CPG module with the same period and offset, and with a
  // smaller order
  MMAddModule( cpg, getPeriod(), getOffset(), getOrder() - 1 );

  StateMachine::init();

}

void WalkMachine::activate ( void ) {

  int i;

  // Grab the position controllers and the encoder reader
  for ( i = 0; i < 6; i++ ) {

    MMGrabModule( control[i], this );

    // Save the old gains and set the new ones
    control[i]->getGains ( &(oldGains[i]) );
    control[i]->setGains ( &(curParams.gains[i]) );

  }
 
  done = false;

  StateMachine::activate();
}

void WalkMachine::deactivate ( void ) {

  int i;

  for ( i = 0; i < 6; i++ ) {

    // Revert to the old controller gains
    control[i]->setGains ( &(oldGains[i]) );

    MMReleaseModule( control[i], this );
  }

  MMReleaseModule( cpg, this );

  StateMachine::deactivate();
}

void WalkMachine::uninit ( void ) {

  // Remove the CPG module
  MMRemoveModule( cpg );

}

// WalkMachine::setSpeedCommand : Sets the current speed command

void WalkMachine::setSpeedCommand( float cmd ) {

  speedCommand = saturate( cmd, -1.0, 1.0 );

}

// WalkMachine::setTurnCommand : Set the current turn command 

void WalkMachine::setTurnCommand( float cmd ) {

  turnCommand = saturate( cmd, -1.0, 1.0 );
}

// WalkMachine::setupProfiles : 
//
// Gets called at each half cycle and sets up the CPG profiles to
// generate leg reference trajectories

void WalkMachine::setupProfiles( void ) {

  int i;
  bool inplace = false;
  float offsets[6];
  float turnOffset;

  if ( speedCommand > 0.25 ) {
    // Going forward ----------------------------

    for ( i = 0; i < 6; i++ )
      cpgProf[i].setModeForward();

    turnOffset = curParams.turnOffset;

  } else if ( speedCommand < -0.25 ) {
    // Going backward ---------------------------

    for ( i = 0; i < 6; i++ )
      cpgProf[i].setModeBackward();

    turnOffset = -curParams.turnOffset;

  } else if ( turnCommand > 0.25 ) {
    // Turning in place to the right -------------

    for ( i = 0; i < 3; i++ ) 
      cpgProf[i].setModeForward(); 

    for ( i = 3; i < 6; i++ ) 
      cpgProf[i].setModeBackward(); 

    inplace = true;

  } else  if ( turnCommand < -0.25 ) {
    // Turning in place to the left --------------

    for ( i = 0; i < 3; i++ ) 
      cpgProf[i].setModeBackward(); 

    for ( i = 3; i < 6; i++ ) 
      cpgProf[i].setModeForward(); 

    inplace = true;
  }
  
  // set last position for continuity of profiler
  for ( i = 0; i < 6; i++ ) {

    cpgProf[i].setPosBegin();
    cpgProf[i].setPosHalfway();
  }

  // If we are turning in place, no offset adjustment is necessary
  for ( i = 0; i < 6; i++ ) {

    if ( inplace ) {

      offsets[i] = 0.0;

    } else {

      offsets[i] = curParams.legOffset[i];

    }
  }

  // Setup the profiles for all legs
  for ( i = 0; i < 6; i++ )

    cpgProf[i].setParams ( curParams.dutyFactor[i], 
                           curParams.sweepAngle[i],
                           offsets[i] );

  // are we turning? if so, reset cpg profier
  if ( !inplace && turnCommand > 0.25 ) {
    // Differential turning to the right ----------

    for ( i = 0; i < 3; i++ ) // left side

      cpgProf[i].setParams (curParams.dutyFactor[i] + curParams.turnDutyFactor,
                            curParams.sweepAngle[i] + curParams.turnSweepAngle,
                            offsets[i] - turnOffset );

    for ( i = 3; i < 6; i++ ) //right side

      cpgProf[i].setParams (curParams.dutyFactor[i] - curParams.turnDutyFactor,
                            curParams.sweepAngle[i] - curParams.turnSweepAngle,
                            offsets[i] + turnOffset );

  } else if ( !inplace && turnCommand < -0.25 ) { // left turn
    // Differential turning to the left ----------

    for ( i = 0; i < 3; i++ ) // left side

      cpgProf[i].setParams (curParams.dutyFactor[i] - curParams.turnDutyFactor,
                            curParams.sweepAngle[i] - curParams.turnSweepAngle,
                            offsets[i] + turnOffset );

    for ( i = 3; i < 6; i++ ) // right side

      cpgProf[i].setParams (curParams.dutyFactor[i] + curParams.turnDutyFactor,
                            curParams.sweepAngle[i] + curParams.turnSweepAngle,
                            offsets[i] - turnOffset );
  }

  for ( i = 0; i < 6; i++ )
    cpgProf[i].setUpFunctions();

}

// WalkMachine::useNextParams : 
//
// Forces the parameters set by the setParams() method to take effect

void WalkMachine::useNextParams( void ) {

  // If there is a pending new parameter set, use it
  if ( nextParams )
    curParams = *nextParams;

  // Record the last commands if we are not about to stop
  if ( speedCommand != 0.0 || turnCommand != 0.0 ) {

    lastSpeedCommand = speedCommand;
    lastTurnCommand = turnCommand;
  }

  cpg->setPeriod(curParams.cpgPeriod);
}

// WalkMachine::gotoAngle : A utility to set an angle target 
//
// The direction parameter determines the encforced rotational direction
// of the resulting leg profile. +1 and -1 result in positive and negative
// leg angle rotation, while 0 uses the closest direction of travel.

void WalkMachine::gotoAngle( uint leg, double time, 
							 float angle, int direction ) {

  MotorTarget_t curTarget;
  double now = MMReadTime();
  float twoPi = 2.0 * M_PI;

  control[leg]->getTarget( &curTarget );

  if ( direction == 0 ) {
    // Choose the closest path

    if ( fabs( angle - curTarget.pos ) > M_PI) {

      if ( angle > curTarget.pos )

        angle -= twoPi;

      else

        curTarget.pos -= twoPi;
    }

  } else if ( direction > 0 ) {

    // If the target angle is smaller, we need to adjust it by 2*M_PI
    while ( angle < curTarget.pos )
      angle += twoPi;
    
  } else {

    // If the target angle is larger, we need to adjust it by 2*M_PI
    while ( angle > curTarget.pos )
      angle -= twoPi;
  }

  prof[leg].setup ( now, now + time, curTarget.pos, angle );

}

// WalkMachine::trackCurrentProfile : 
//
// Commands the motors to track the currently setup profile for each
// leg ( used for lifting and bringing down the tripods )

void WalkMachine::trackCurrentProfile ( void ) {

  int i;
  MotorTarget_t tar;

  for ( i = 0; i < 6; i++ ) {

    tar = prof[i].getVal();
    control[i]->setTarget ( &tar );

  }
}

// WalkMachine::trackCPGProfile : 
//
// Commands the motors to track the currently setup CPG profile for
// each leg (used for the actual walking

void WalkMachine::trackCPGProfile ( void ) {
   
  int i;
  double c = cpg->getVal();
  MotorTarget_t tar;

  for ( i = 1; i < 6; i += 2 ) {

    tar = cpgProf[i].getVal ( c + 0.5 );
    // We need to scale the velocity with the period of the CPG to obtain the 
    // real desired velocity
    tar.vel = tar.vel / curParams.cpgPeriod;
    control[i]->setTarget ( & tar );
  }

  for ( i = 0; i < 6; i += 2 ) {

    tar = cpgProf[i].getVal ( c );
    // We need to scale the velocity with the period of the CPG to obtain the 
    // real desired velocity
    tar.vel = tar.vel / curParams.cpgPeriod;
    control[i]->setTarget ( &tar );
  }
}
