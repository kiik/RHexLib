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
 * $Id: enc_test_mm.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Example program to test the low level EncoderHW interface through
 * the use of the module manager.
 *
 * Created       : Uluc Saranli, 01/16/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the Hardware class encoder interfaces in
// RHexLib. On execution, the program continuously prints the encoder
// reads for all 6 axex. The following keyboard commands are
// implemented:
//
// 'r' : reset all encoders
// 'd' : Disable all encoders
// 'e' : Enable all encoders
// 
// All other keys exit the program
//
// Note: This example does not function with the virtual hardware
// because the virtual hardware requires the module manager to be
// active to function.
// ==========================================================================

#include <stdio.h>
#include "sysutil.hh"
#include "ModuleManager.hh"

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

// Define a module that will print the values of the encoders.
class PrintModule : public Module {

public:
  PrintModule ( void ) 
    : Module( "print", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };

  void  activate ( void ) {
    int axis;

    for ( axis = 0; axis < 6; axis++ )
      hw.encoders->enable( axis );    // Enable all 6 encoders
  };

  void  deactivate ( void ) {
    int axis;

    for ( axis = 0; axis < 6; axis++ )
      hw.encoders->disable( axis );   // Disable all 6 encoders
  };

  void  update ( void ) {
    int axis;

    printf( "Encoders: " );
    for ( axis = 0; axis < 6; axis++ )
      printf("0x%04x, ", hw.encoders->read( axis ) );

    printf( "\n" );
  }
};

// Define a module to look for user input
class UserModule : public Module {
public:
  UserModule ( void ) 
    : Module( "userinput", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };

  void  update ( void ) {
    int inp;
    int axis;

    // Check for user input and do the appropriate thing.
    if ( kbhit() ) {
      inp = getch();
      switch ( inp ) {
      case 'r':
        for ( axis = 0; axis < 6; axis++ )
          hw.encoders->reset( axis );     // Reset all encoders
        break;

      case 'd':
        for ( axis = 0; axis < 6; axis++ )
          hw.encoders->disable( axis );   // Disable all encoders
        break;

      case 'e':
        for ( axis = 0; axis < 6; axis++ )
          hw.encoders->enable( axis );    // Enable all encoders
        break;

      default: // Exit on any other keystroke
        MMMessage( "User interrupt: Shutting down!\n" );
        MMPowerOff( );
      }
    }
  }
};

int main( void ) {

  PrintModule pr;
  UserModule ui;

  // Let the module manager know about the current hardware */
  MMChooseHardware( &hw );

  // Add and activate the print module
  MMAddModule( &pr, 10, 0, 10 );
  MMActivateModule( &pr );

  // Add and activate the user input module.
  MMAddModule( &ui, 50, 0, 10 );
  MMActivateModule( &ui );

  // Call the Module Manager main loop
  MMMainLoop();

  MMPrintModules();

  // Normally, the MMMainLoop() does not exit, but shutdown the system here,
  //   just in case...
  MMShutdown();


  return 0;
}
