
/*
 *
 * RHEXLIB 
 *
 * WalkExp.hh
 *
 * ExpRunner for various walking behaviors.
 *
 * Created by: Uluc Saranli
 * Created on: 01/25/2001
 * Last Modified by: Robert Peters
 * Last Modified on: 2001/06/26
 *
 * This file and all of RHEXLIB are copyright 2000 by the University
 * of Michigan. Copying and/or distributing require the expressed
 * written consent of the authors or maintainers of this code.
 *
 */

#ifndef _WALKEXP_HH
#define _WALKEXP_HH

// ==========================================================================
// WalkExP class
//
// This is the ExperimentRunner derivative, which sets up and runs the batch
// experiments. The current user interface is setup as follows:
//
// Overall, pulling the left level left aborts the experiment batch.
//
// ***** Experiment setup
// - Triggered by pushing the right lever forward
// - Runs the StartupMachine, which calibrates the robot and stands up.
// - The standup machine is configured to use the leg offset adjustments.
//
// ***** Experiment phase
// - Triggered by pushing the right lever forward
// - The robot continuously goes forward, with left right turning control only
// - Also, depending on the offset control mode, the R/C knob gives leg offset
// control
// - The R/C switch, pulled forward starts a timer.
// - The R/C switch, pushed backward stops the timer.
// - The experiment ends when the left lever is pushed forward.
// 
// ***** Wrapup phase
// - The wrapup phase uses the Sitmachine to sit down.
// - Once the SitDown is complete, it waits for user input to indicate 
//   experiment success or failure.
// - Right lever is pushed right indicated success,  
//   pushed left, indicates failure.
// - The wrapup finishes after the user input and the next experiment proceeds.
//

#include "ExpRunner.hh"
#include "WalkMachine.hh"
#include "RHexWalker.hh"
#include "SitMachine.hh"
#include "StartupMachine.hh"
#include "RemoteControl.hh"
#include "PositionControl.hh"
#include "CalibMachine.hh"
#include "AnalogOutput.hh"
#include "EncoderReader.hh"
#include "Hardware.hh"
#include "basicmath.hh"


// delays for the RC sticks ( in seconds )
#define RIGHT_RC_STICK 0
#define RIGHT_RC_XDIAL 1
#define RIGHT_RC_YDIAL 0
#define RIGHT_STICK_DELAY 0.005

#define LEFT_RC_STICK 1
#define LEFT_RC_XDIAL 3
#define LEFT_RC_YDIAL 2
#define LEFT_STICK_DELAY 0.1

// -----------------------------------------------------------------
// TestExperiment class. Provides the body pitch in radians ------- 

class WalkExp : public ExpRunner {
public:

  WalkExp( ) : ExpRunner( "walkexp", 0 ) { };
  /* Interface to the particular experiment definition ------- */

  /* State change events */
  bool setupBeginCheck( int num );
  bool experimentBeginCheck( int num );
  bool experimentEndCheck( int num ); 
  bool wrapupEndCheck( int num );
  bool terminateCheck( int num );
  bool abortCheck( int num ); /* Event to abort the batch */
  bool repeatCheck( int num );

  /* State action methods */
  void initEntry( int num );
  void idleEntry( int num );
  void setupEntry( int num );
  void setupDuring( int num );
  void experimentEntry( int num );
  void experimentDuring( int num );
  void experimentExit( int num );
  void wrapupExit( int num );
  void handleAbort( int num );
  void handleRepeat( int num );

  // Module functionality
  void init( void );

  //  void setFilterParams( FilterParam_t *p ) { filterParams = *p; };

  bool walkIsDone( void );
  void setParameterNames( void );
  void computeParameterValues( uint num );


private:

  // Modules used by the WalkExp
  StartupMachine * sm;
  SitMachine * sitm;
  WalkMachine * wm;
  RHexWalker * rw;
  RemoteControl *rc;
  Hardware * hw;
  CalibMachine *calib[6];

  // Mark for timing
  CLOCK timerMark;
  CLOCK timerMark2;
  CLOCK timerVal;
  CLOCK walkMark;

  // Results of the experiment
  int walkSpeed;
  bool expSuccess;

  WalkParam_t  walkParams;
  Floats       paramValues;

  bool secondPhaseStart;

  // these are the parameter defaults for the walking machine
  WalkParam_t curParams;

  // Parameter sweep configuration.
  int    repeats, cur_repeat, numofexps;
  Floats cpgperiod_range;
  int    cpgperiod_cur;
  Floats sweepangle_range;
  int    sweepangle_cur;
  Floats dutyfactor_range;
  int    dutyfactor_cur;
  Floats legoffset_range;
  int    legoffset_cur;
  Floats smooth_range;
  int    smooth_cur;
  Floats turnoffset_range;
  int    turnoffset_cur;
  Floats turnduty_range;
  int    turnduty_cur;
  Floats turnsweep_range;
  int    turnsweep_cur;
  Floats kp_range;
  int    kp_cur;
  Floats kd_range;
  int    kd_cur;

};

#endif
