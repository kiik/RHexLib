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
 * $Id: SitMachine.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/03/2001
 * Last Modified : Uluc Saranli, 07/05/2001
 *
 ********************************************************************/

#include "ModuleManager.hh"
#include "StdModules.hh"
#include "SitMachine.hh"
#include "PositionControl.hh"

#define OWNER ((SitMachine *) owner)

// Events ------------------------------------------------------------

bool SitMachine::DoneSitting::check ( void ) { 

  return bool( MMReadTime() >= OWNER->mark + OWNER->timeSit );

}
// States -------------------------------------------------------------

void SitMachine::Sit::entry ( void ) { 

  int i;

  // Record the current time
  OWNER->mark = MMReadTime(); 

  for ( i = 0; i < 6; i++ ) {

    // Determine the current position of the leg and the appropriate 
    // direction of travel
    double pos = OWNER->enc[i]->getPosition();

    // Command the leg to go to the appropriate angle
    if ( pos > 0.25 * M_PI ) 

      OWNER->gotoAngle( i, OWNER->timeSit, 1.5 * M_PI );

    else 

      OWNER->gotoAngle( i, OWNER->timeSit, -0.5 * M_PI );
  }
}

void SitMachine::Sit::during ( void ) {

  // Use the position controllers to track the currently setup profile
  OWNER->trackCurrentProfile();

}

void SitMachine::Sit::exit ( void ) { 

  OWNER->done = true;

};

void SitMachine::SitDone::during ( void ) {}

// StandMachine Methods --------------------------------------------------

SitMachine::SitMachine ( void ) : StateMachine( SITMACHINE_NAME ) {

  // allocate events
  doneSitting = new DoneSitting ( this );

  // allocate states
  sit = new Sit ( this );
  sitDone = new SitDone ( this );

  // add transitions ( this defines the structure of the machine )
  // 
  //           From State      Event         To State
  //------------------------------------------------------------
  Transition ( sit,            doneSitting,  sitDone );

  // define state machine
  initialize ( sit );

  // set up parameters -- this should eventually be done with uluc's
  // configuration utility
  timeSit = MMGetFloatSymbol ( "sit_time", 0.4 );
  
}

SitMachine::~SitMachine () {

  if ( doneSitting ) delete ( doneSitting );

  if ( sit ) delete ( sit );
  if ( sitDone ) delete ( sitDone );

}

void SitMachine::init ( void ) {

  int i;

  for ( i = 0; i < 6; i++ )
    if ( ( control[i] = (MotorControl *) 
           MMFindModule( POSITIONCONTROL_NAME, i )) == NULL)
      MMFatalError ( "StandMachine::init", "Cannot find motor controller!" );

  for ( i = 0; i < 6; i++ )
    if ( ( enc[i] = (EncoderReader *) 
           MMFindModule( ENCODERREADER_NAME, i )) == NULL)
      MMFatalError ( "StandMachine::init", "Cannot find encoder sensor!" );

  StateMachine::init();
}

void SitMachine::activate ( void ) {

  int i;
  MotorGains_t gains;

  // Read in the default parameter set
  gains.kp = MMGetFloatSymbol ( "walk_kp", 24.015 );
  gains.kd = MMGetFloatSymbol ( "walk_kd", 0.67242 );

  // Grab and activate the controller and encoder modules
  for ( i = 0; i < 6; i++ ) {

    MMGrabModule ( control[i], this );
    control[i]->setGains ( & gains );

    MMGrabModule ( enc[i], this );
  }
 
  done = false;

  StateMachine::activate();

}

void SitMachine::deactivate ( void ) {

  int i;

  // Release the modules grabbed during activation
  for ( i = 0; i < 6; i++ ) {
    MMReleaseModule ( control[i], this );
    MMReleaseModule ( enc[i], this );
  }
 
}

// SitMachine::gotoAngle : A utility to set an angle target
void SitMachine::gotoAngle( uint leg, double time, float angle ) {

  MotorTarget_t curTarget;
  double now = MMReadTime();

  control[leg]->getTarget( &curTarget );

  prof[leg].setup ( now, now + time, curTarget.pos, angle );
}

// SitMachine::trackCurrentProfile : Tracks the currently setup leg
// profile. Must be called by the during() function of the states for
// setting motor targets
void SitMachine::trackCurrentProfile ( void ) {

  int i;
  MotorTarget_t tar;

  for ( i = 0; i < 6; i++ ) {

    tar = prof[i].getVal();
    control[i]->setTarget ( &tar );

  }

}
