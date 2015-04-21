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
 * $Id: rc_test.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * ModuleManager class function definitions
 *
 * Example program to test the remote control functionality
 *
 * Created       : Uluc Saranli, 04/18/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the functionality of the remote controls and
// dials Upon execution, it runs for 20 seconds, logging the valus of
// the dials and the output of the RemoteControl modules.
//
// Pressing any key before the 20 second elapses exits the program.
//
// Note: 
//
// time
// dial 0 value
// dial 1 value
// dial 2 value
// dial 3 value
// dial 4 value
// dial 5 value
// RC left stick X
// RC left stick Y
// RC right stick X
// RC right stick Y
//
// ==========================================================================

#include <stdio.h>
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif
#include "sysutil.hh"
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "RemoteControl.hh"
#include "DataLogger.hh"

// If we are running QNX, determine which Hardware we should use.
// Note that the RHEX_HARDWARE environment variable determines this.
#ifdef _QNX4_

#ifdef _MICHIGAN_
#include "MichiganHW.hh"
MichiganHW hw;
#endif

#ifdef _MCGILL_
#include "McGillHW.hh"
McGillHW hw;
#endif
#endif // #ifdef _QNX4_

#ifdef _LINUX_
#include "VirtualHW.hh"
#include "SimSectHW.hh"
//VirtualHW       hw;
SimSectHW       hw;
#endif

/* -------------------------------------------------------------------------  */
// DataLogger class to log position and velocity information 
// for all legs.

class LoggerModule : public DataLogger {
public:
  LoggerModule( void ) : DataLogger ( 11, "loggermodule", 0 ) { };

  void init( void ) {
    if ( ( rc = ( RemoteControl * ) 
           MMFindModule( "remotecontrol", 0 )) == NULL)
        MMFatalError ( "LoggerModule::init", "Cannot find remote control!" );

	DataLogger::init();
  };

  void  fillRecord ( float *f ) {
    int           i, count;

    count = 0;

    // Record the time
    f[count++] = MMReadTime();

    for ( i = 0; i < 6; i++ ) {
      // Record the dial readings
      f[ count++ ] = hw.dials->read(i);
    }
    f[ count++ ] = rc->readStickX( 0 );
    f[ count++ ] = rc->readStickY( 0 );
    f[ count++ ] = rc->readStickX( 1 );
    f[ count++ ] = rc->readStickY( 1 );
  };

private:

  RemoteControl *rc;

};

// ------------------------------------------------------------------------- 
// Timer module to exit after a certain period */

class TimerModule : public Module {
public:
  TimerModule ( void ) 
    : Module( "timer", 0, false, false ) { };

  void  init ( void ) {
    if ( ( logger = ( LoggerModule * ) 
           MMFindModule( "loggermodule", 0 )) == NULL)
      MMFatalError ( "TimerModule::init", "Cannot find data logger module!" );
  };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {

    if ( MMReadTime() > 20 || kbhit() ) {
      // Time is up ( or user interrupt ). Save data and exit

      MMMessage( "Time is up: Shutting down!\n" );

      // We need to shutdown and disable the engines before saving the data
      MMShutdown( );

      // Write the collected data to a file
      logger->exportToFile( "rc_test.data", DL_ASCII );
    
      // Exit the program.
      MMPowerOff( );
    }
  };

private:

  LoggerModule *logger;

};

// -------------------------------------------------------------------------
// Main entry point to the program

int main ( void ) {

  TimerModule      timer;
  LoggerModule     logger;
  RemoteControl    rc( "remotecontrol" );

#ifdef _QNX4_
  setprio( getpid(), 23 );
#endif

  // Let the module manager know about the hardware to use
  MMChooseHardware( &hw );

  // Add all the standard RHex modules
  RHexAddStdModules();

  // Module creation ------------------------------------------

  // Adding the modules ---------------------------------------

  // Add the logger, timer and the controller  modules
  MMAddModule ( & rc, 1, 0, USER_DEFINED_SENSORS );
  MMAddModule ( & logger, 1, 0, LOGGING_MODULES );
  MMAddModule ( & timer, 50, 0, OTHER_MODULES );

  rc.configureStick( 0, 1, 0.3, 3, 2 );
  rc.configureStick( 1, 1, 0.005, 1, 0 );

  // Module activation ---------------------------------------

  // Activate the logger and the timer modules
  MMActivateModule( & logger );
  MMActivateModule( & timer );
  MMActivateModule( & rc );

  // Invoke the module manager main loop ---------------------
  MMMainLoop();

  // Normally, execution never reaches here.
  MMShutdown();

  return 0;
}
