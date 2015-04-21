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
 * $Id: PushupController.cc,v 1.2 2001/08/14 03:08:52 ulucs Exp $
 *
 * The PushupController Module.
 *
 * Created       : Uluc Saranli, 06/01/2000
 * Last Modified : Uluc Saranli, 07/13/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "PushupController.hh"
#include "StdModules.hh"
#include "PositionControl.hh"

void PushupController::init( void ) {
  int i;

  // Acquire pointers to all 6 position controllers
  for ( i = 0; i < 6; i ++ ) {
    if ( ( control[i] = ( PositionControl * )
           MMFindModule( POSITIONCONTROL_NAME, i )) == NULL)
      MMFatalError ( "PushupController::init", "Cannot find motor controller!" );
  }

  lowerTime = MMGetFloatSymbol( "pushup_lower_time" , 1.0 );
  chillTime = MMGetFloatSymbol( "pushup_chill_time" , 0.3 );
  standTime = MMGetFloatSymbol( "pushup_stand_time" , 0.4 );
  waitTime = MMGetFloatSymbol( "pushup_wait_time" , 0.3 );
}

void PushupController::activate( void ) {
  int          i;
  MotorGains_t gains;

  // Start from phase 0: start lowering the robot
  phase = 0;

  // Get exclusive access to all the position controllers
  for ( i = 0; i < 6; i++ )
    MMGrabModule( control[i], this );

  // Save the old PD gains and set the new gains for all
  // 6 position controllers.
  gains.kp = 5.0;
  gains.kd = 0.14;

  for ( i = 0; i < 6; i++ ) {
    control[i]->getGains( & oldGains[i] );
    control[i]->setGains( & gains );
  }
}

void PushupController::deactivate( void ) {

  int          i;

  // Restore the old gains for the position controllers
  for ( i = 0; i < 6; i++ )
    control[i]->setGains( & oldGains[i] );

  // Release all the position controllers for other users
  for ( i = 0; i < 6; i++ )
    MMReleaseModule( control[i], this );

}

void PushupController::update( void ) {

  double now = MMReadTime();

  // Perform necessary tasks based on the current phase 
  switch ( phase ) {

  case 0:   // Starting to lower the robot

    // Setup leg profile to lower the robot
    legProfiler.setup( now, now + lowerTime, 0, M_PI / 2.5 );

    mark = now;
    phase = 1;

    break;

  case 1:   // Waiting for the robot to lower

    // Track the current leg profile
    if ( now < ( mark + lowerTime ) )
      trackCurrentProfile();
    else {
      mark = now;
      phase = 2;
    }

    break;

  case 2:   // Wait for a while in the sitting position

    if ( now > ( mark + chillTime ) ) {
      mark = now;
      phase = 3;
    }
    break;

  case 3:   // Starting to standup

    // Setup leg profile to raise the robot
    legProfiler.setup( now, now + standTime, M_PI / 2.5, 0 );

    mark = now;
    phase = 4;

    break;

  case 4:   // Waiting for the robot to standup

    // Track the current leg profile
    if ( now < ( mark + standTime ) )
      trackCurrentProfile();
    else {
      mark = now;
      phase = 5;
    }

    break;

  case 5:   // Wait for a while in the standing position

    if ( now > ( mark + waitTime ) ) {
      mark = now;
      phase = 0;
    }
    break;

  }
}

void PushupController::trackCurrentProfile ( void ) {

  int i;
  MotorTarget_t target;

  // Find out where the legs should be right now
  target = legProfiler.getVal();

  // Command all 6 motor controllers to go to that position.
  for ( i = 0; i < 6; i++ ) {
    if(i==0 or i==3) continue;
    control[i]->setTarget ( & target );
  }
}

