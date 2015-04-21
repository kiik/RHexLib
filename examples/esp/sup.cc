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
 * $Id: sup.cc,v 1.1 2001/08/06 17:01:44 ulucs Exp $
 *
 * Example program to test various RHex behaviors.
 *
 * Created       : Eric Klavins, 01/03/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// ==========================================================================
// This program uses the Supervisor machine to control various
// behaviors of RHex. The Supervisor module accepts input through the
// remote control and performs calibration, standup, sitdown, walking
// and turning behaviors.
//
// Any key exits the program
// ========================================================================== 

#include <stdio.h>
#include "sysutil.hh"
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "StateMachine.hh"
#include "Supervisor.hh"

// If we are running QNX, use VirtualHW, otherwise, use SimSect.
#ifdef _QNX4_
#include "VirtualHW.hh"
VirtualHW hw;
#endif

#ifdef _LINUX_
#include "SimSectHW.hh"
SimSectHW hw;
#endif

class UserModule : public Module {

  public:

    UserModule ( void ) : Module( "userinput", 0, false, false ) { };

    void  init ( void ) {}
    void  uninit ( void ) {}
    void  activate ( void ) {}
    void  deactivate ( void ) {}
    void  update ( void ) {

      int i;

      if ( kbhit() ) {

        // Disable all motor drives
        for ( i = 0 ; i < 6 ; i++) 
          hw.driveEnable ( i, false );

        MMPowerOff();
      }

    }
} umod;

int main ( void ) {

  // Read the system specific configuration file 
  MMReadConfigFile( "rhex_simsect.rc" );

  // Read the application dependent configuration file
  MMReadConfigFile( "sup.rc" );

  MMChooseHardware ( & hw );

  Supervisor supervisor;

  // Create and add all the standard modules
  RHexAddStdModules();

  // Add the user module and supervisor
  MMAddModule ( &umod, 10, 0, USER_CONTROLLERS );
  MMAddModule ( &supervisor, 1, 0, USER_CONTROLLERS );

  // Activate the supervisor modules
  MMActivateModule ( &supervisor );

  // Activate the user module
  MMActivateModule( &umod );

  // Call the Module manager main loop
  MMMainLoop();

  // This part is never reached as the main loop only exits when the
  // program ends.
  MMShutdown();
  return 0;

}







