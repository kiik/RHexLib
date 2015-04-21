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
 * $Id: drive_test.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Example program to test the low level EncoderHW interface
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program tests the Hardware class drive enable interface of
// RHexLib. The following keyboard commands are implemented:
//
// 'd' : Disable drives
// 'e' : Enable drives
// 
// All other keys exit the program
// ==========================================================================

#include <stdio.h>
#include "sysutil.hh"

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

int main( void ) {
  int axis;
  int done = 0;
  char inp;

  // This is necessary in the abscence of a call to MMChooseHardware()
  hw.initialize();

  // Set all the analog outputs to 6V
  for ( axis = 0; axis < 6; axis++ )
    hw.analogIO->write( axis, 5.7 );

  // Initially, disable all the drives
  for ( axis = 0; axis < 6; axis++ )
    hw.driveEnable( axis, false );

  while ( !done ) {

    if ( kbhit() ) {
      inp = getch();

      switch ( inp ) {

      case 'd': // Disable the drives

        for ( axis = 0; axis < 6; axis++ )
          hw.driveEnable( axis, false );
        break;

      case 'e': // Enable the drives

        for ( axis = 0; axis < 6; axis++ )
          hw.driveEnable( axis, true );
        break;

      default:
        done = 1;
      }
    }
  }

  // Disable all drives before exiting
  for ( axis = 0; axis < 6; axis++ )
    hw.driveEnable( axis, true );

  // This is necessary in the abscence of a call to MMChooseHardware()
  hw.cleanup();

  return 0;
}
