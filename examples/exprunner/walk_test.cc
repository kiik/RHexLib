/*
 *
 * RHEXLIB
 *
 * walk_test.cc
 *
 * ExperimentRunner for various walking behaviors.
 *
 * Created by: Uluc Saranli
 * Created on: 01/25/2001
 * Last Modified by: Uluc Saranli
 * Last Modified on: 01/25/2001
 *
 * This file and all of RHEXLIB are copyright 2000 by the University
 * of Michigan. Copying and/or distributing require the expressed
 * written consent of the authors or maintainers of this code.
 *
 */

/* ========================================================================== */
/* This program uses the ExperimentRunner facility to run batch walking
 * experiments and log the results.
 *
 * Any key exits the program */
/* ========================================================================== */

#include <stdio.h>
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif
#include "sysutil.hh"
#include "StdModules.hh"
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "StandMachine.hh"
#include "SitMachine.hh"
#include "CalibMachine.hh"
#include "AnalogOutput.hh"
#include "EncoderReader.hh"
#include "PositionControl.hh"
#include "DataLogger.hh"
#include "StallSensor.hh"
#include "Profiler.hh"
#include "WalkMachine.hh"
#include "RHexWalker.hh"
#include "StartupMachine.hh"
#include "ExpRunner.hh"
#include "RemoteControl.hh"
#include "WalkExp.hh"
#include "RHexLogger.hh"

#ifdef _QNX4_

#ifdef _MICHIGAN_
#include "MichiganHW.hh"
MichiganHW hw;
#endif

#ifdef _MCGILL_
#include "McGillHW.hh"
McGillHW hw;
#endif

#endif

#ifdef _LINUX_
#include "SimSectHW.hh"
SimSectHW hw;
#endif

// Global module pointers for easy access
// EncoderReader   * encoder[6];
// AnalogOutput    * aout[6];
// SpeedFilter     * filter[6];
// PositionControl * control[6];
// StallSensor     * stallSensor[6];

WalkExp * tester;

class SafetyModule : public Module {
public:
  SafetyModule ( void ) : Module( "safety", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {
    if ( kbhit() ) {
      MMMessage( "User interrupt: Shutting down!\n" );
      MMPowerOff( );
    }
    if ( tester->isDone() ) {
      MMMessage( "Experiments done: Shutting down!\n" );
      MMPowerOff( );
    }
  }
};

int main ( void ) {

  SafetyModule   sf;
  RemoteControl *rc;

#ifdef _QNX4_
  //  setprio( getpid(), 23 );
#endif

  // Read the hardware specific configuration file based on which
  // platform the compilation is taking place.
#ifdef _QNX4_

#ifdef _MICHIGAN_
  MMReadConfigFile( "rhex_michigan.rc" );
#endif

#ifdef _MCGILL_
  MMReadConfigFile( "rhex_mcgill.rc" );
#endif
#endif // #ifdef _QNX4_

#ifdef _LINUX_
  MMReadConfigFile( "rhex_simsect.rc" );
#endif

  MMReadConfigFile( "walk_test.rc" );


  MMChooseHardware ( & hw );

  tester = new WalkExp(  );

  RHexAddStdModules();
  MMAddModule( &sf, 50, 0, 50 );
  MMAddModule( tester, 1, 0, 120 );

  MMActivateModule( &sf );
  MMActivateModule( tester );

  rc = (RemoteControl*) MMFindModule( REMOTECONTROL_NAME, 0 );

  if (rc == NULL)
	MMFatalError("walk_test", "rhexrc not found");

  MMActivateModule( rc );

  MMMainLoop();

  MMShutdown();

  return 0;

}
