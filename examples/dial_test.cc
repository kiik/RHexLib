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
 * $Id: dial_test.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Example program to test the low level DialHW and SwitchHW interfaces
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the Hardware class dial and switch interfaces of
// RHexLib. On execution, the program continuously prints the values
// for the first 6 dials and the first 8 switches.
//
// Note that the module manager needs to be operational in this case
// because the dials are actually read from PWM inputs.
//
// Any key exits the program
// ==========================================================================

#include <stdio.h>
#include "sysutil.hh"
#include "types.hh"
#include "ModuleManager.hh"
#include "RHexLogger.hh"
#include "SensorSuite.hh"

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
#include "SimSectHW.hh"
SimSectHW hw;
#endif

// Test Logger class ----------------------------------------------------
// Logs all 6 dials and 8 switches --------------------------------------

class TestLogger : public DataLogger {
public:
  TestLogger( void ) : DataLogger( 15, "testlogger", 0 ) { };

  void fillRecord( float *f ) {
    int i;

    // Record the time
    f[0] = MMReadTime();

    // Record all 6 dial channels
    for ( i = 0; i < 6; i++ )
      f[i + 1] = hw.dials->read( i );

    // Record all 8 switches
    for ( i = 0; i < 8; i++ )
      f[i + 7] = hw.switches->read( i );
  }
};
TestLogger *testLogger = NULL;

// PrintModule class ----------------------------------------------------------
// Define a module that will print the values of the dials and switches. ------

class PrintModule : public Module {

public:
  PrintModule ( void ) 
    : Module( "print", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {
    int i;

    printf( "Dials: " );
    for ( i = 0; i < 6; i++ )
      printf( "% 2.3f,", hw.dials->read( i ) );

    printf( "Switches: " );
    for ( i = 0; i < 8; i++ )
      printf( "%i,", hw.switches->read( i ) );

    printf( "\n" );
  }
};

// UserModule class ----------------------------------------------------------
// Looks for user input and shutdown the system when necessary ---------------

class UserModule : public Module {
public:
  UserModule ( void ) 
    : Module( "userinput", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {
    if ( kbhit() ) {
      MMMessage( "User interrupt: Shutting down!\n" );

      // Save the logged data if logging was enabled at the beginning
      if ( testLogger != NULL ) {

        MMShutdown();
        MMMessage( "Saving data...\n" );
        testLogger->exportToFile( "dial_test" );
      }

      MMPowerOff( );
    }
  }
};

int main ( void ) {

  PrintModule pr;
  UserModule ui;

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

  // Read the application dependent configuration file
  MMReadConfigFile( "dial_test.rc" );

  // Let the module manager know about the current hardware
  MMChooseHardware( &hw );

  // Add and activate the print module
  MMAddModule( &pr, 50, 0, 10 );
  MMActivateModule( &pr );

  // Add and activate the user input module.
  MMAddModule( &ui, 50, 0, 10 );
  MMActivateModule( &ui );

  // If logging is enabled, create, add and activate the logger
  if ( MMGetFloatSymbol( "logging_enable", 0.0 ) == 1.0 ) {
    int logging_period;

    MMMessage( "Logging is enabled. Adding and activating the logger.\n" );
    testLogger = new TestLogger;

    // Retrieve the logging period from the configuration file
    logging_period = int( MMGetFloatSymbol( "logging_period", 1 ) );

    MMAddModule( testLogger, logging_period, 0, 10 );
    MMActivateModule( testLogger );
  }

  MMPrintModules();

  // Call the Module Manager main loop
  MMMainLoop();

  // Normally, the MMMainLoop() does not exit, but shutdown the system here,
  // just in case...
  MMShutdown();

  return 0;

}
