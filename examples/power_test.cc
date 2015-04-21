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
 * $Id: power_test.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Example program to test the low level PowerHW input interface
 *
 * Created       : Uluc Saranli, 01/23/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the Hardware class power measurement interface
// of RHexLib. On execution, the program continuously prints the
// values for the battery voltage and current.
//
// Any key exits the program
// ==========================================================================

#include <stdio.h>
#include "sysutil.hh"
#include "types.hh"
#include "ModuleManager.hh"
#include "SensorSuite.hh"
#include "DataLogger.hh"
#include "AnalogOutput.hh"

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

// Analog input Sensor objects -----------------------------------------------
BatteryVoltSensor *voltage;
BatteryCurrSensor *current;

// Testing related setting variables
bool      MotorLoadActive;      // Switch to activate the motor load
bool      BatteryFilterActive;  // Switch to activate the battery value filters
float     APEXCommandVoltage;   // Voltage to be applied to the APEX drive
                                // for motor loading

// Test Logger class ---------------------------------------------------------
// Logs all battery voltage and current --------------------------------------

class TestLogger : public DataLogger {
public:
  TestLogger( void ) : DataLogger( 3, "testlogger", 0 ) { };

  void fillRecord( float *f ) {
    // Record the time
    f[0] = MMReadTime();

    // Record the filtered voltage and current
    f[1] = voltage->value();
    f[2] = current->value();
  }
};
TestLogger *testLogger = NULL;

// PrintModule class ----------------------------------------------------------
// Define a module that will print the values of the dials and switches.

class PrintModule : public Module {

public:
  PrintModule ( void ) 
    : Module( "print", 0, false, false ) { };

  void  init ( void ) { 
    // Create the input sensors
    voltage = new BatteryVoltSensor( &hw );
    current = new BatteryCurrSensor( &hw );

    // Setup internal filter
    Fc = 2;   // Cut-off in Hz
    Fs = 20;  // Update rate of the update routine of this module
    setFilterCoeff( );  

    // Initialize the internal variables
    Vx_1 = 0.0;
    Vy_1 = 0.0;
    Ix_1 = 0.0;
    Iy_1 = 0.0;
  };

  void  uninit ( void ) { 
    // Delete the Analog input sensors
    delete voltage;
    delete current;
  };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void ) {
    char msg[128];

    filterBatteryReading( );
    sprintf( msg, "Battery voltage: %2.4f, Battery current: %2.4f\n", 
            voltage_value, current_value );
    MMMessage( msg );
  };

  void setFilterCoeff ( ) {
    c1 = 1 / ( 1 + 2 * Fs / Fc );
    c2 = ( 1 - 2 * Fs / Fc ) * c1;
  };

  void filterBatteryReading ( void ) {

    if ( ! BatteryFilterActive ) {  

      voltage_value = voltage->value ( );
      current_value = current->value ( );

    } else {

      voltage_value = c1 * ( voltage->value ( ) + Vx_1 ) - c2 * Vy_1;
      Vy_1 = voltage_value;
      Vx_1 = voltage->value ( );
      current_value = c1 * ( current->value ( ) + Ix_1 ) - c2 * Iy_1;
      Iy_1 = current_value;
      Ix_1 = current->value ( );
    }
  };

private:
  // Internal filter related variables

  float Fc;
  float Fs;
  float c1, c2;
  float voltage_value;
  float current_value;
  float Vy_1, Vx_1;
  float Iy_1, Ix_1;
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
    int i;
    if ( kbhit() ) {

      // Diable all motor drives first
      for ( i = 0 ; i < 6 ; i++) hw.driveEnable ( i, false );

      MMMessage( "User interrupt: Shutting down!\n" );

      // Save the logged data if logging was enabled at the beginning
      if ( testLogger != NULL ) {

        MMShutdown();
        MMMessage( "Saving data...\n" );
        testLogger->exportToFile( "power_test" );
      }

      MMPowerOff( );
    }
  }
};


// Motor command module to test different load conditions

class MotorCommand : public Module {
public:
  MotorCommand ( void )
    : Module ( "motorcommand", 0, SINGLE_USER, PERIODIC_UPDATE ) { };

  void  init ( void ) { 
    aout = new AnalogOutput ( 2 );
    MMAddModule ( aout, 1, 0, 100 );
  };
  void  uninit ( void ) { };
  void  activate ( void ) { 
    MMGrabModule ( aout, this );
  };
  void  deactivate ( void ) { 
    MMReleaseModule ( aout, this );
  };
  void  update ( void ) {
    aout->setValue ( APEXCommandVoltage );
  };
private:
  AnalogOutput * aout;
};

MotorCommand motor;

int main ( void ) {

  PrintModule pr;
  UserModule  ui; 
   
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
  MMReadConfigFile( "power_test.rc" );


  // Set the testing related global variables
  if ( MMGetFloatSymbol ( "motor_loading_active", 0.0 ) != 0.0 )
    MotorLoadActive = true;
  else
    MotorLoadActive = false;

  if ( MMGetFloatSymbol ( "battery_filters_active", 0.0 ) != 0.0 )
    BatteryFilterActive = true;
  else
    BatteryFilterActive = false;

  APEXCommandVoltage = MMGetFloatSymbol ( "apex_command_voltage", 6.0 );

  // Let the module manager know about the current hardware
  MMChooseHardware( &hw );

  // Add and activate the print module
  MMAddModule( &pr, 50, 0, 10 );
  MMActivateModule( &pr );

  // Add and activate the user input module.
  MMAddModule( &ui, 50, 0, 10 );
  MMActivateModule( &ui );

  // Add the motor command module for loading
  if ( MotorLoadActive ) {

    MMAddModule( & motor, 1, 0, 50 );
    MMActivateModule( & motor );
  }

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

  // Enable the motors
  if ( MotorLoadActive )
    hw.driveEnable ( 1, true );
    
  // Call the Module Manager main loop
  MMMainLoop();

  // Normally, the MMMainLoop() does not exit, but shutdown the system here,
  // just in case...
  MMShutdown();

  return 0;

}
