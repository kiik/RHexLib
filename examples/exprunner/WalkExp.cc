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
 * $Id: WalkExp.cc,v 1.2 2001/08/14 03:09:27 ulucs Exp $
 *
 * An example experiment runner for the walking behavior.
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 08/12/2001
 *
 ********************************************************************/

#include <math.h>
#include "WalkExp.hh"

#ifdef _QNX4_
#pragma off(unreferenced)
#endif

// Transition events =======================================================
// Trigger to start the Setup phase ---------------------

bool WalkExp::setupBeginCheck( int num ) {
  float xval, yval;
  //  float xval2, yval2;

  // The left lever left push.
  rc->readStick( 1, &xval, &yval );

  return bool( xval < -0.5 );
}

// Trigger to start the Experiment phase ----------------
bool WalkExp::experimentBeginCheck( int num ) {
  float xval, yval;

  // The left lever right push
  rc->readStick( 1, &xval, &yval );

  return bool( secondPhaseStart && rw->isDone() && xval > 0.5 );
}

// Trigger to end the Experiment phase ------------------
bool WalkExp::experimentEndCheck( int num ) {
  return wm->isDone();
}

// Trigger to start the Wrapup phase --------------------
bool WalkExp::wrapupEndCheck( int num ) {
  float xval, yval;

  // The end condition for the wrapup is the sitmachine to have
  // completed its thing, and the right lever state to indicate
  // success or failure.
  rc->readStick( 0, &xval, &yval );

  return bool( sitm->isDone() && ( xval < -0.5 || xval > 0.5 ) );
}

// Trigger for end of the batch -------------------------
bool WalkExp::terminateCheck( int num ) {

  return bool( num == numofexps );
}

// Trigger to abort the batch in an emergency -----------
bool WalkExp::abortCheck( int num ) { 
  float xval, yval;
  float xval2, yval2;

  // The left lever left push.
  rc->readStick( 1, &xval, &yval );
  rc->readStick( 0, &xval2, &yval2 );

  return bool( yval < -0.5 );
};

bool WalkExp::repeatCheck( int num ) {

  float xval, yval;

  rc->readStick( 1, &xval, &yval );
  return bool( yval > 0.5 );
}


// Actions for different phases  ============================================

// -------------------------------------------------
// Entry to the idle phase ------------------------
void WalkExp::initEntry( int num ) {

  ExpRunner::setLogFlag( true );
}

// -------------------------------------------------
// Entry to the idle phase ------------------------
void WalkExp::idleEntry( int num ) {

  // Turn off the LED to indicate entry to the idle state.
  hw->digitalIO->setBit( 0, 6, false );
}

// ------------------------------------------------- 
// Entry to the setup phase ------------------------ 
//Here you should compute the parameters to run from the experiment
//number, and setup any modules you're using to use those parameters
void WalkExp::setupEntry( int num ) {
  int i;
  char msg[128];

  secondPhaseStart = false;

  // Invalidate all calibration
  for ( i = 0; i < 6; i++ )
    calib[i]->reset();

  // Compute and choose the current parameter values.
  computeParameterValues( num );

  // Output an informative message with the parameter values.
  sprintf( msg , "%03d: ", num );
  MMMessage( msg );

  for ( i = 0; i < 10; i++ ) {
    sprintf( msg , "%02.2f ", paramValues.get( i ) );
    MMMessage( msg );
  }
  MMMessage( "\n" );

  //prepare for setup part 2

#ifdef _QNX4_
  MMGrabModule( sm, this );
#endif

#ifdef _LINUX_
  MMGrabModule( rw, this );
  //  rw->setGait( RHexWalker::TRIWHEEL_GAIT );
  secondPhaseStart = true;
#endif

}

// -------------------------------------------------
// During the setup phase --------------------------
// Used to prepare Rhex if RHex needs to be in a certain state before
// the experiment can start
void WalkExp::setupDuring( int num ) {
  int i;
  float temp;

  if ( !secondPhaseStart ) {

    //  MMMessage("I'm in the first phase");
    if ( sm->getStatus() == StartupMachine::SUCCESS ) {
      MMReleaseModule ( sm, this );

      //prepare for setup part 2
      secondPhaseStart = true;

      // Activate the walking controller with full forward speed
      MMGrabModule( rw, this );
	  //      rw->setGait( RHexWalker::TRIWHEEL_GAIT );

    } else if ( sm->getStatus() == StartupMachine::FAILURE ) {

      // We need to restart the startup machine when the user prompts
      float xval, yval;

      // The right lever forward push
      rc->readStick( 0, &xval, &yval );
      if ( yval > 0.5 ) {
        MMReleaseModule ( sm, this );

        for ( i = 0; i < 6; i++ )
          calib[i]->reset();           // invalidate all calibration
    
        MMGrabModule ( sm, this );
      }
    }
  } else {

    // MMMessage("I'm in the second phase");

    // Tell the walking controller about the speed and turning user inputs
    //  wm->setSpeedCommand( rc->readStickY( RIGHT_RC_STICK ) );
    temp = hw->dials->read( 4 );

    rw->setForwardCommand( (temp + 1) / 2 * rc->readStickY( RIGHT_RC_STICK ));
    rw->setTurnCommand( (temp + 1) / 2 * rc->readStickX( RIGHT_RC_STICK ) );

  }
}

// -------------------------------------------------
// Entry to the experiment phase -------------------
// Here you should start up your experiment (i.e. the walking machine)
// and any last parameters
void WalkExp::experimentEntry( int num ) {

  
  // Stop the RHexWalker
  MMReleaseModule( rw, this );

  // Start the bare-bones WalkMachine at full forward speed
  MMGrabModule( wm, this );
  wm->setSpeedCommand( 1.0 );

  // Set the current walking parameters. Note that telling the
  // Walkmachine about the parameters once here will be sufficient
  // because it retrieves the parameters from the necessary field when
  // necessary.
  wm->setParams( &curParams );

  // Reset the timer
  timerVal = timerMark = timerMark2 = 0;

  // Turn on the LED to indicate experiment progress
  hw->digitalIO->setBit( 0, 6, true );
}

// -------------------------------------------------
// During the experiment phase ---------------------
// Put in any control logic for while the experiment is running
void WalkExp::experimentDuring( int num ) {
  float xval, yval;
  CLOCK now = MMReadClock();

  // The right lever forward push indicates end of experiment
  rc->readStick( 0, &xval, &yval );
  if ( xval < 0.25 && xval > -0.25 && yval > 0.5 ) {
    // Stop the walking machine. This will be detected by the
    // experimentEndCheck()
    wm->setSpeedCommand( 0.0 );
    wm->setTurnCommand( 0.0 );
  }

  // Set the turning controls of the walking machine
  //  rc->readStick( 0, &xval, &yval );
  //  wm->setTurnCommand( xval );

  // Check the R/C switch to see whether we started the timer...
  if ( hw->dials->read( 5 ) > 0.5 && timerMark == 0 ) {

    // Check if a minimum amount of time elapsed before changing timer
    // state
    if ( timerMark2 != 0 && ( now - timerMark2 > 50000 ) ) {
      // Timer started. Mark the time
      timerMark = now;
      timerMark2 = 0;
      timerVal = 0;
      MMMessage( "Timer started\n" );
    } else if ( timerMark2 == 0 ) {
      timerMark2 = now;
    }
  } else if ( hw->dials->read( 5 ) < -0.5 && timerMark != 0 ) {

    // Check if a minimum amount of time elapsed before changing timer state
    if ( timerMark2 != 0 && ( now - timerMark2 > 50000 ) ) {
      // Timer ended. Mark the time
      timerVal = now - timerMark;
      timerMark = 0;
      timerMark2 = 0;
      MMMessage( "Timer stopped\n" );
    } else if ( timerMark2 == 0 ) {
      timerMark2 = now;
    }
  }
}

// -------------------------------------------------
// Exit from the experiment phase ------------------
// Stop your experiment and start any shutdown procedures
void WalkExp::experimentExit( int num ) {

  // Release the WalkMachine
  MMReleaseModule ( wm, this );

  // activate the SitMachine
  MMGrabModule( sitm, this );
}

// -------------------------------------------------
// Exit from the wrapup phase ------------------
// Make sure the post experiment "things" are done
void WalkExp::wrapupExit( int num ) {
  int i;
  char log[2048];
  float xval, yval;

  rc->readStick( 0, &xval, &yval );
  if ( xval < -0.5 )
    expSuccess = false;
  else  
    expSuccess = true;

  // Fill in the last necessary field before letting ExpRunner know
  // about the current set of parameters.
  paramValues.set( 10, expSuccess );
  ExpRunner::setParameterValues( paramValues );

  for ( i = 0; i < 6; i++ )  // disable motor drives
    hw->driveEnable( i, false );
  MMReleaseModule ( sitm, this );

  // Write out the parameter values to the log file
  logMessage( log );
}

// -------------------------------------------------
// Handling experiment abort signal ----------------
// If an abort signal is sent do what?
// Typically shut down whatever is running, and prepare to start again.
void WalkExp::handleAbort( int num ) {
  int i;

  // Turn off the experiment LED indicator
  hw->digitalIO->setBit( 0, 6, false );

  for ( i = 0; i < 6; i++ )  // disable motor drives
    hw->driveEnable( i, false );
  
  switch ( getStatus() ) {
    // Might need to shut down wm in SETUP...
  case SETUP:
    if ( secondPhaseStart )
      MMReleaseModule ( rw, this );
    else
      MMReleaseModule ( sm, this );
    break;

  case RUNNING:
    MMReleaseModule ( wm, this );
    break;

  case WRAPUP:
    MMReleaseModule ( sitm, this );
    break;
  
  default:
    break;

  }
}

// -------------------------------------------------
// Handling experiment repeat signal ----------------
// If an repeat signal is sent do what?  
// Typically shut down whatever is running, and prepare to start again.
void WalkExp::handleRepeat( int num ) {
  handleAbort( num );
}


// Module methods  ===================================================---

void WalkExp::init( void ) {
  Floats dflt(3);
  int i;

  // Search for the needed modules and set up pointers to them

  if ( ( sm = ( StartupMachine * ) 
         MMFindModule( "startupmachine", 0 ) ) == NULL )
    MMFatalError ( "WalkExperiment::init", "Cannot find Startup Machine" );
  
  if ( ( sitm = ( SitMachine * ) MMFindModule( "sitmachine", 0 ) ) == NULL )
    MMFatalError ( "WalkExperiment::init", "Cannot find Sit Machine" );
  
  if ( ( wm = ( WalkMachine * ) MMFindModule( "walkmachine", 0 ) ) == NULL )
    MMFatalError ( "WalkExperiment::init", "Cannot find Walk Machine" );

  if ( ( rw = ( RHexWalker * ) MMFindModule( "rhexwalker", 0 ) ) == NULL )
    MMFatalError ( "WalkExperiment::init", "Cannot find RHex Walker" );

  if ( ( rc = ( RemoteControl * ) MMFindModule( "rhexrc", 0 ) ) == NULL )
    MMFatalError ( "WalkExperiment::init", "Cannot find remote control unit" );

  for ( i = 0; i < 6; i++ ) {
    if ( ( calib[i] = ( CalibMachine * ) 
           MMFindModule( "calibmachine", i ) ) == NULL )
      MMFatalError ( "WalkExperiment::init", 
                     "Cannot find calibration machine" );
  }

  hw = MMGetHardware();

  // Retrieve parameter sweep configuration info from the symbol table
  dflt.set( 0, 0.0 );

  cpgperiod_range = MMGetArraySymbol( "walkexp_cpgperiod_range" );
  if ( cpgperiod_range.getCount() != 3 ) 
    MMFatalError("WalkExp::init", 
                 "cpgperiod_range must be properly defined!");
  sweepangle_range = MMGetArraySymbol( "walkexp_sweepangle_range" );
  if ( sweepangle_range.getCount() != 3 ) 
    MMFatalError("WalkExp::init", 
                 "sweepangle_range must be properly defined!");
  dutyfactor_range = MMGetArraySymbol( "walkexp_dutyfactor_range" );
  if ( dutyfactor_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "dutyfactor_range_range must be properly defined!");
  legoffset_range = MMGetArraySymbol( "walkexp_legoffset_range" );
  if ( legoffset_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "legoffset_range must be properly defined!");
  smooth_range = MMGetArraySymbol( "walkexp_smooth_range" );
  if ( smooth_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "smooth_range must be properly defined!");
  turnoffset_range = MMGetArraySymbol( "walkexp_turnoffset_range" );
  if ( turnoffset_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "turnoffset_range must be properly defined!");
  turnduty_range = MMGetArraySymbol( "walkexp_turnduty_range" );
  if ( turnduty_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "turnduty_range must be properly defined!");
  turnsweep_range = MMGetArraySymbol( "walkexp_turnsweep_range" );
  if ( turnsweep_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "turnsweep_range must be properly defined!");
  kp_range = MMGetArraySymbol( "walkexp_kp_range" );
  if ( kp_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "kp_range must be properly defined!");
  kd_range = MMGetArraySymbol( "walkexp_kd_range" );
  if ( kd_range.getCount() != 3 )
    MMFatalError("WalkExp::init", 
                 "kd_range must be properly defined!");

  curParams.tripodTime = WALK_TRIPODTIME_DFLT;
  curParams.standAdjTime = WALK_ADJTIME_DFLT;
  curParams.cubic = 1.0;

  curParams.cpgPeriod = cpgperiod_range.get( 1 );
  curParams.smooth = smooth_range.get( 1 );
  curParams.turnOffset = turnoffset_range.get( 1 );
  curParams.turnSweepAngle = turnsweep_range.get( 1 );
  curParams.turnDutyFactor = turnduty_range.get( 1 );
  for ( i = 0; i < 6; i++ ) {
    curParams.dutyFactor[i] = dutyfactor_range.get( 1 );
    curParams.legOffset[i] = legoffset_range.get( 1 );
    curParams.sweepAngle[i] = sweepangle_range.get( 1 );
    curParams.gains[i].kp = kp_range.get( 1 );
    curParams.gains[i].kd = kd_range.get( 1 );
    curParams.gains[i].ka = 0.0;
  }

  // Number of times an experiment will be repeated.
  repeats = int( MMGetFloatSymbol( "walkexp_repeats", 1.0 ) );

  numofexps = 
    int ( repeats *
		  cpgperiod_range.get( 0 ) *
		  sweepangle_range.get( 0 ) *
		  dutyfactor_range.get( 0 ) *
		  legoffset_range.get( 0 ) *
		  smooth_range.get( 0 ) *
		  turnoffset_range.get( 0 ) *
		  turnduty_range.get( 0 ) *
		  turnsweep_range.get( 0 ) *
		  kp_range.get( 0 ) *
		  kd_range.get( 0 ) );
    
  timerMark = timerMark2 = timerVal = walkMark = 0;

  rc->configureStick( 0, 1, 0.1, 1, 0 );
  rc->configureStick( 1, 1, 0.1, 3, 2 );

  // Set the names of the parameters to be logged.
  setParameterNames();

  // Call the init of the base module
  ExpRunner::init();
}

bool WalkExp::walkIsDone() {

  return ExpRunner::isDone();
}

void WalkExp::setParameterNames( void ) {

  Strings names( 11 );

  names.set(0, "cpg_period");
  names.set(1, "sweep_angle");
  names.set(2, "duty_factor");
  names.set(3, "leg_offset");
  names.set(4, "smooth_factor");
  names.set(5, "turn_leg_offset");
  names.set(6, "turn_duty_factor");
  names.set(7, "turn_sweep_angle");
  names.set(8, "kp");
  names.set(9, "kd");
  names.set(10, "success");

  ExpRunner::setParameters( names );
}

void WalkExp::computeParameterValues( uint num ) {
  Floats values( 11 );
  int i, tempnum;

  // The following computation cycles through the parameters and
  // chooses which index experiment needs to be run. The order of
  // cycling is as follows:
  //  cpgperiod;
  //  sweepangle
  //  dutyfactor
  //  legoffset
  //  smooth
  //  turnoffset
  //  turnduty
  //  turnsweep
  //  kp
  //  kd
  cur_repeat = num % repeats;
  tempnum = num / repeats;

  cpgperiod_cur = tempnum % int( cpgperiod_range.get( 0 ) );
  tempnum = tempnum / int( cpgperiod_range.get( 0 ) );

  sweepangle_cur = tempnum % int( sweepangle_range.get( 0 ) );
  tempnum = tempnum / int( sweepangle_range.get( 0 ) );

  dutyfactor_cur = tempnum % int( dutyfactor_range.get( 0 ) );
  tempnum = tempnum / int( dutyfactor_range.get( 0 ) );

  legoffset_cur = tempnum % int( legoffset_range.get( 0 ) );
  tempnum = tempnum / int( legoffset_range.get( 0 ) );

  smooth_cur = tempnum % int( smooth_range.get( 0 ) );
  tempnum = tempnum / int( smooth_range.get( 0 ) );

  turnoffset_cur = tempnum % int( turnoffset_range.get( 0 ) );
  tempnum = tempnum / int( turnoffset_range.get( 0 ) );

  turnduty_cur = tempnum % int( turnduty_range.get( 0 ) );
  tempnum = tempnum / int( turnduty_range.get( 0 ) );

  turnsweep_cur = tempnum % int( turnsweep_range.get( 0 ) );
  tempnum = tempnum / int( turnsweep_range.get( 0 ) );

  kp_cur = tempnum % int( kp_range.get( 0 ) );
  tempnum = tempnum / int( kp_range.get( 0 ) );

  kd_cur = tempnum % int( kd_range.get( 0 ) );
  tempnum = tempnum / int( kd_range.get( 0 ) );

  // The following block sets the walking parameter values based on
  // the current sweep index. Note that if the number of specified
  // sweep experiments is 1, this block does not set the parameter and
  // relies on the init() function to set the value.
  if ( cpgperiod_range.get( 0 ) != 1 )
    curParams.cpgPeriod 
      = cpgperiod_range.get( 1 ) 
        + cpgperiod_cur 
          * ( cpgperiod_range.get( 2 ) - cpgperiod_range.get( 1 ))
          / ( cpgperiod_range.get( 0 ) - 1 );

  if ( smooth_range.get( 0 ) != 1 )
    curParams.smooth 
      = smooth_range.get( 1 ) 
        + smooth_cur 
          * ( smooth_range.get( 2 ) - smooth_range.get( 1 ))
          / ( smooth_range.get( 0 ) - 1 );

  if ( turnoffset_range.get( 0 ) != 1 )
    curParams.turnOffset 
      = turnoffset_range.get( 1 ) 
        + turnoffset_cur 
          * ( turnoffset_range.get( 2 ) - turnoffset_range.get( 1 ))
          / ( turnoffset_range.get( 0 ) - 1 );

  if ( turnsweep_range.get( 0 ) != 1 )
    curParams.turnSweepAngle 
      = turnsweep_range.get( 1 ) 
        + turnsweep_cur 
          * ( turnsweep_range.get( 2 ) - turnsweep_range.get( 1 ))
          / ( turnsweep_range.get( 0 ) - 1 );

  if ( turnduty_range.get( 0 ) != 1 )
    curParams.turnDutyFactor 
      = turnduty_range.get( 1 ) 
        + turnduty_cur 
          * ( turnduty_range.get( 2 ) - turnduty_range.get( 1 ))
          / ( turnduty_range.get( 0 ) - 1 );

  for ( i = 0; i < 6; i++ ) {
    if ( dutyfactor_range.get( 0 ) != 1 )
      curParams.dutyFactor[i] 
        = dutyfactor_range.get( 1 ) 
          + dutyfactor_cur 
            * ( dutyfactor_range.get( 2 ) - dutyfactor_range.get( 1 ))
            / ( dutyfactor_range.get( 0 ) - 1 );

    if ( sweepangle_range.get( 0 ) != 1 )
      curParams.sweepAngle[i] 
        = sweepangle_range.get( 1 ) 
          + sweepangle_cur 
            * ( sweepangle_range.get( 2 ) - sweepangle_range.get( 1 ))
            / ( sweepangle_range.get( 0 ) - 1 );

    if ( legoffset_range.get( 0 ) != 1 )
      curParams.legOffset[i] 
        = legoffset_range.get( 1 ) 
          + legoffset_cur 
            * ( legoffset_range.get( 2 ) - legoffset_range.get( 1 ))
            / ( legoffset_range.get( 0 ) - 1 );

    if ( kp_range.get( 0 ) != 1 )
      curParams.gains[i].kp 
        = kp_range.get( 1 ) 
          + kp_cur 
            * ( kp_range.get( 2 ) - kp_range.get( 1 ))
            / ( kp_range.get( 0 ) - 1 );

    if ( kd_range.get( 0 ) != 1 )
      curParams.gains[i].kd 
        = kd_range.get( 1 ) 
          + kp_cur 
            * ( kd_range.get( 2 ) - kd_range.get( 1 ))
            / ( kd_range.get( 0 ) - 1 );

  }

  // Inform the Experiment runner about the values of the parameters
  // that we have chosen.
  values.set( 0, curParams.cpgPeriod );
  values.set( 1, curParams.sweepAngle[0] );
  values.set( 2, curParams.dutyFactor[0] );
  values.set( 3, curParams.legOffset[0] );
  values.set( 4, curParams.smooth );
  values.set( 5, curParams.turnOffset );
  values.set( 6, curParams.turnSweepAngle );
  values.set( 7, curParams.turnDutyFactor );
  values.set( 8, curParams.gains[0].kp );
  values.set( 9, curParams.gains[0].kd );

  paramValues = values;

}

