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
 * $Id: poscontrol_test.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Example program to test the PositionControl module and its functionality
 *
 * Created       : Uluc Saranli, 03/22/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the functionality of the PositionControl module.
// Upon execution, it runs for 5 seconds, commanding triangular
// profiles to each of the six legs.
//
// Pressing any key before the 5 second elapses exits the program.
//
// Note: The program logs the desired and actual positions and
// velocities of all the legs in the file poscontrol_test.data. The
// file is organized in columns, where the column order is:
//
// motor 0 actual position
// motor 0 desired position
// motor 0 actual velocity
// motor 0 desired velocity
// motor 1 actual position
// motor 1 desired position
// motor 1 actual velocity
// motor 1 desired velocity
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
#include "StdModules.hh"
#include "AnalogOutput.hh"
#include "EncoderReader.hh"
#include "PositionControl.hh"
#include "RHexLogger.hh"

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
// Timer module to exit after a certain period -----------------------------

class TimerModule : public Module {
public:
  TimerModule ( void ) 
    : Module( "timer", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {

    if ( MMReadTime() > 5 || kbhit() ) {
      // Time is up ( or user interrupt ). Save data and exit

      MMMessage( "Time is up: Shutting down!\n" );

      // Save the logged data if logging was enabled at the beginning
      if ( MMGetFloatSymbol( "logging_enable", 0.0 ) == 1.0 ) {

        MMShutdown();
        MMMessage( "Saving data...\n" );
        rhexlogger->exportToFile( );
      }

      // Exit the program.
      MMPowerOff( );
    }
  };
};

// -------------------------------------------------------------------------
// Controller module to command trajectories to each leg

class ControllerModule : public Module {
public:
  ControllerModule( void ) :
    Module( "controller", 0, false, false ) { };

  void  init ( void ) { 
    int i;

    // Locate all the modules that we are going to use.
    for ( i = 0; i < 6; i++ )
      if ( ( control[i] = ( PositionControl * ) 
             MMFindModule( POSITIONCONTROL_NAME, i )) == NULL)
        MMFatalError ( "ControllerModule::init", 
                       "Cannot find motor controller!" );

    // Initialize internal variables.
    for ( i = 0; i < 6; i++ ) {
      position[i] = 0.0;
      velocity[i] = 0.0;
    }
  };

  void  uninit ( void ) { };

  void  activate ( void ) { 
    int          i;
    MotorGains_t newGains;
    double       now = MMReadTime();

    newGains.kp = 24.03;
    newGains.kd = 0.6727;
    
    // Acquire ownership of the PositionControl modules we will use.
    for ( i = 0; i < 6; i++ ) {

      control[i]->setErrorOffset( M_PI/1.3 );
      MMGrabModule( control[i], this );

      control[i]->getGains( &oldGains[i] ); // Save the old motor gains
      control[i]->setGains( &newGains ); // Set the motor gains to what we want

      // Mark the current time plus a different small offset for each motor
      // to obtain different phases for the triangular trajectories.
      mark[i] = now + 0.2 * i;

      // Set the velocity target of each motor to a different value
      velocity[i] = 3.0 + 3.0 * i;

      hw.driveEnable( i, true );
    }
  };

  void  deactivate ( void ) { 
    int i;

    // Release ownership of the previously grabbed modules.
    for ( i = 0; i < 6; i++ ) {
      MMReleaseModule( control[i], this );

      control[i]->setGains( &oldGains[i] ); // Revert back to the old gains.

      hw.driveEnable( i, false );
    }
  };

  void  update ( void ) {
    int            i;
    MotorTarget_t  target;
    double         now = MMReadTime();

    // Update the targets for each motor
    for ( i = 0; i < 6; i++ ) {
      // Update the trajectory profile
      position[i] += velocity[i] * CLOCK_TO_SEC( MMGetStepPeriod() );

      target.pos = position[i];
      target.vel = velocity[i];
      target.acc = 0.0;

      control[i]->setTarget( &target );
    }

    // Check to see if any of the motors needs to change direction
    for ( i = 0; i < 6; i++ ) {
      if ( now > mark[i] + 2 ) {
        // Reverse direction and update the time mark
        velocity[i] = -velocity[i];
        mark[i] += 2;
      }
    }   
  };

private:

  PositionControl *control[6];
  MotorGains_t    oldGains[6];
  float           position[6];
  float           velocity[6];
  double          mark[6];

};

// -------------------------------------------------------------------------
// Main entry point to the program

int main ( void ) {

  TimerModule      timer;
  ControllerModule controller;

#ifdef _MICHIGAN_
#ifdef _QNX4_
  setprio( getpid(), 23 );
#endif
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
  MMReadConfigFile( "poscontrol_test.rc" );

  // Let the module manager know about the hardware to use
  MMChooseHardware( &hw );

  // Add all the standard RHex modules
  RHexAddStdModules();

  // Add the timer and the controller  modules
  MMAddModule ( & timer, 50, 0, OTHER_MODULES );
  MMAddModule ( & controller, 1, 0, BEHAVIORAL_CONTROLLERS );

  // Activate the timer module
  MMActivateModule( & timer );

  // Activate the controller (which in turn, grabs and activates 
  // the position controllers ).
  MMActivateModule( & controller );

  // If logging is enabled, create, add and activate the logger
  if ( MMGetFloatSymbol( "logging_enable", 0.0 ) == 1.0 ) {

    MMMessage( "Logging is enabled. Activating the logger.\n" );

    MMActivateModule( rhexlogger );
  }

  // Invoke the module manager main loop ---------------------
  MMMainLoop();

  // Normally, execution never reaches here.
  MMShutdown();

  return 0;
}
