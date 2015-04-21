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
 * $Id: backemf_test.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Example program to test the low level motor interface by applying constant
 * voltage to the motors and logging the speed, back EMF etc.
 *
 * Created       : Uluc Saranli, 03/23/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the functionality of the low level motor
// interface.  Upon execution, it first calibrates the motors to
// determine the correct drive offset. Then, it runs for a few
// seconds, commanding different voltages to different motors.
//
// Pressing any key before the end exits the program.
//
// Note: The program logs the motor positions and velocities, as well
// as the voltage command to the DCMotorHW interface and readings of
// motor terminal voltage, current and back EMF.
//
// motor 0 actual position
// motor 0 actual velocity
// motor 0 requested command
// motor 0 measured (or estimated) motor terminal voltage
// motor 0 measured (or estimated) motor armature current
// motor 0 measured (or estimated) motor back EMF
// motor 1 actual position
// ... and so on for all the 6 motors
//
// ========================================================================== 

#include <stdio.h>
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif
#include "sysutil.hh"
#include "ModuleManager.hh"
#include "StdModules.hh"   // This header file declares all the standard global modules.
#include "AnalogOutput.hh"
#include "EncoderReader.hh"
#include "PositionControl.hh"
#include "CalibMachine.hh"
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
#include "SimSectHW.hh"
SimSectHW       hw;
#endif

// ------------------------------------------------------------------------- 
// DataLogger class to log position and velocity information 
// for all legs.

class TestLogger : public DataLogger {
public:
  TestLogger( void ) : DataLogger ( 36, "testlogger", 0 ) { };

  void init( void ) {
    int i;

    // Locate all the modules that we are going to use.  This is in
    // general how pointers to different modules in the system are
    // obtained.
    for ( i = 0; i < 6; i++ )
      if ( ( encoder[i] = ( EncoderReader * ) 
             MMFindModule( ENCODERREADER_NAME, i )) == NULL)
        MMFatalError ( "LoggerModule::LoggerModule", 
                       "Cannot find encoder reader!" );

	DataLogger::init();
  };

  void activate( void ) {
    int i;

    // Grab the encoder modules to activate them
    for ( i = 0; i < 6; i++ )
      MMGrabModule( encoder[i], this );
  }

  void deactivate( void ) {
    int i;

    // Release the encoder modules
    for ( i = 0; i < 6; i++ )
      MMReleaseModule( encoder[i], this );
  }

  void  fillRecord ( float *f ) {
    int           i, count;

    count = 0;

    // Record the time
    f[count++] = MMReadTime();

    for ( i = 0; i < 6; i++ ) {

      // Record the actual position and velocity
      f[ count++ ] = encreader[i]->getPosition();
      f[ count++ ] = encreader[i]->getSpeed();

      // Record the command to the motor
      f[ count++ ] = hw.dcmotors->getCommand( i );;

      // Record the measured ( or estimated ) motor states
      f[ count++ ] = hw.dcmotors->getVoltage( i );
      f[ count++ ] = hw.dcmotors->getCurrent( i );
      f[ count++ ] = hw.dcmotors->getBackEMF( i );
    }
  };

private:

  EncoderReader    *encoder[6];

};
TestLogger *testLogger = NULL;

// -------------------------------------------------------------------------
// Timer module to exit after a certain period

class TimerModule : public Module {
public:
  TimerModule ( void ) 
    : Module( "timer", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { mark = MMReadTime(); };
  void  deactivate ( void ) { };
  void  update ( void ) {

    if ( MMReadTime() > ( mark + 4 ) || kbhit() ) {
      // Time is up ( or user interrupt ). Save data and exit

      MMMessage( "Time is up: Shutting down!\n" );

      // Save the logged data if logging was enabled at the beginning
      if ( testLogger != NULL ) {

        MMShutdown();
        MMMessage( "Saving data...\n" );
        testLogger->exportToFile( "backemf_test" );
      }

      // Exit the program.
      MMPowerOff( );
    }
  };

private:

  double  mark;

};
TimerModule *timer;

// ------------------------------------------------------------------------- 
// Controller module to command voltages on each leg

class ControllerModule : public Module {
public:
  ControllerModule( void ) :
    Module( "controller", 0, false, false ) { };

  void  init ( void ) { 
    int i;

    // Locate the calibration modules that we are going to use.
    for ( i = 0; i < 6; i++ )
      if ( ( calib[i] = ( CalibMachine * ) 
             MMFindModule( CALIBMACHINE_NAME, i )) == NULL)
        MMFatalError ( "ControllerModule::init", 
                       "Cannot find calibration machine!" );

    // Initialize internal variables.
    for ( i = 0; i < 6; i++ ) {
      voltage[i] = 0.0;
    }

    // Start from the uncalibrated state.
    calibState = false;
  };

  void  uninit ( void ) { };

  void  activate ( void ) { 
    int          i;

    for ( i = 0; i < 6; i++ ) {

      // Set the calibration mode to manual
      calib[i]->setMode( CalibMachine::MANUAL );
      // Grab and activate calibration machines
      MMGrabModule( calib[i], this );

    }
  };

  void  deactivate ( void ) { 
    int i;

    // Disable all the drives
    for ( i = 0; i < 6; i++ )
      hw.driveEnable( i, false );
  };

  void  update ( void ) {
    int     i;
    double  now = MMReadTime();

    if ( !calibState ) {
      // Calibration is still going on. Wait for the calibration
      // machines to finish

      // Check the states of all calibration machines.
      calibState = true;
      for ( i = 0; i < 6; i++ )
        if ( calib[i]->getStatus() != CalibMachine::SUCCESS ) 
          calibState = false;

      if ( calibState ) {
        // All the calibration machines are done! Initialize the
        // voltage controls

        for ( i = 0; i < 6; i++ ) {
          // Release and deactivate calibration machines
          MMReleaseModule( calib[i], this );

          // Mark the current time plus a different small offset for each motor
          // to obtain different phases for the trajectories.
          mark[i] = now + 0.2 * i;

          // Set the velocity target of each motor to a different value
          voltage[i] = 2.0 + 2.0 * i;

          // Enable all motor drives
          hw.driveEnable( i, true );
        }

        MMMessage( "Starting the back EMF tests.\n" );

        if ( testLogger ) {
          // Activate the logger module
          MMGrabModule( testLogger, this );
        }
      }
    } else {
      // The calibration is complete. Proceed with the voltage commands.

      // Update the voltages for each motor
      for ( i = 0; i < 6; i++ )
        hw.dcmotors->setCommand( i, voltage[i] );

      // Check to see if any of the motors needs to change direction
      for ( i = 0; i < 6; i++ ) {

        if ( now > mark[i] + 0.5 ) {

          // Reverse direction of voltages
          voltage[i] = -voltage[i];
          mark[i] += 0.5;
        }
      }   
    }
  };

private:

  CalibMachine *calib[6];
  float voltage[6];
  double mark[6];

  bool  calibState; //Flag to indicate whether calibration is complete or not.
};

// ------------------------------------------------------------------------- 
// Main entry point to the program

int main ( void ) {

  ControllerModule controller;

#ifdef _QNX4_
  setprio( getpid(), 23 );
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

  // Read the application dependent configuration file
  MMReadConfigFile( "backemf_test.rc" );

  // Let the module manager know about the hardware to use
  MMChooseHardware( &hw );

  // Add all the standard RHex modules
  RHexAddStdModules();

  // Module creation ------------------------------------------
  timer = new TimerModule;

  // Adding the modules ---------------------------------------

  // Add the logger, timer and the controller  modules
  MMAddModule ( timer, 50, 0, OTHER_MODULES );
  MMAddModule ( & controller, 1, 0, BEHAVIORAL_CONTROLLERS );

  // Module activation ---------------------------------------

  // Activate the timer 
  MMActivateModule( timer );

  // Activate the controller (which in turn, grabs and activates 
  // the position controllers).
  MMActivateModule( & controller );

  // If logging is enabled, create and add the logger
  if ( MMGetFloatSymbol( "logging_enable", 0.0 ) == 1.0 ) {
    int logging_period;

    MMMessage( "Logging is enabled. Adding and activating the logger.\n" );
    testLogger = new TestLogger;

    // Retrieve the logging period from the configuration file
    logging_period = int( MMGetFloatSymbol( "logging_period", 1 ) );

    MMAddModule( testLogger, logging_period, 0, LOGGING_MODULES );
  }

  // Invoke the module manager main loop ---------------------
  MMMainLoop();

  // Normally, execution never reaches here.
  MMShutdown();

  return 0;
}
