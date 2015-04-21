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
 * $Id: accel_test_filtered.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Example program to test the low level AccelHW interface.
 *
 * Created       : Haldun Komsuoglu, 05/10/2001
 * Last Modified : Haldun Komsuoglu, 05/10/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the low level Hardware interface to the
// accelerometers.  It continuously reads, low-pass filters and prints
// the accelerations along the X, Y and Z axes.
//
// All keys exit the program
//
// Note: This example does not function with the virtual
// hardware. Moreover, the use of the module manager is required
// because PulseWidth modules are used by the Michigan hardware.
//
// ==========================================================================

// System includes
#include <stdio.h>
#include "sysutil.hh"

// Local includes
#include "ModuleManager.hh"
#include "PulseWidth.hh"
#include "Filter.hh"

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
VirtualHW hw;
#endif



Filter         *LPF[3];
FilterParam_t  Fparam; 



// -- Define generic sensor interfaces for all the accelerometers ----
class AccSensor : public GenericSensor {

public:
  AccSensor ( AccelHW::Axis Ch_desired ) { Ch = Ch_desired; }; 

  float value ( void ) { return hw.accels->read ( Ch ); };

private:
  AccelHW::Axis Ch;  // Defines the channel being assigned
};

AccSensor *A_sens[3];




// -- Define a module that will print the values of the encoders. ---
class PrintModule : public Module {

public:
  PrintModule ( void ) 
    : Module( "print", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) {  };
  void  update ( void ) {
    printf("X:% 1.7f, ", LPF[0]->output () );
    printf("Y:% 1.7f, ", LPF[1]->output () );
    printf("Z:% 1.7f\n", LPF[2]->output () );
    /*
    printf("X:% 1.7f, ", hw.accels->read ( AccelHW::AXIS_X ) );
    printf("Y:% 1.7f, ", hw.accels->read ( AccelHW::AXIS_Y ) );
    printf("Z:% 1.7f\n", hw.accels->read ( AccelHW::AXIS_Z ) );
    */
  }
};



// -- Define a module to look for user input -----------------------
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
      MMPowerOff( );
    }
  }
};

int main( void ) {

  int          i;
  PrintModule  pr;
  UserModule   ui;

  // -- Desing the filters -----------------------------
  // Filter parameters
  Fparam.Ftype = RC;
  Fparam.Fc    = 1;
  Fparam.Fs    = 1000;
  Fparam.Gp    = 1;
  // X-axis accelerometer
  LPF[0] = new Filter ( "LPF", 0 );  
  LPF[0]->Design ( &Fparam ); 
  A_sens[0] = new AccSensor ( AccelHW::AXIS_X );
  LPF[0]->AssignSource ( A_sens[0] );
  // Y-axis accelerometer
  LPF[1] = new Filter ( "LPF", 1 );  
  LPF[1]->Design ( &Fparam ); 
  A_sens[1] = new AccSensor ( AccelHW::AXIS_Y );
  LPF[1]->AssignSource ( A_sens[1] );
  // Z-axis accelerometer
  LPF[2] = new Filter ( "LPF", 2 );  
  LPF[2]->Design ( &Fparam ); 
  A_sens[2] = new AccSensor ( AccelHW::AXIS_Z );
  LPF[2]->AssignSource ( A_sens[2] );


  MMReadConfigFile( "accel_test_filtered.rc" );
  MMChooseHardware( &hw );


  // Add and activate the filter modules
  for ( i = 0; i < 3; i++){
    MMAddModule( LPF[i], 1, 0, 5 );
    MMActivateModule( LPF[i] );
  }

  // Add and activate the print module
  MMAddModule( &pr, 10, 0, 10 );
  MMActivateModule( &pr );

  // Add and activate the user input module.
  MMAddModule( &ui, 50, 0, 10 );
  MMActivateModule( &ui );

  MMPrintModules();

  MMMainLoop();

  MMPrintModules();

  MMShutdown();

  return 0;
}
