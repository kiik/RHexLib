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
 * $Id: sup.cc,v 1.4 2001/08/15 17:29:49 ulucs Exp $
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
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif
#include "sysutil.hh"
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "StateMachine.hh"
#include "Supervisor.hh"
#include "RemoteControl.hh"
#include "RHexLogger.hh"
#include "PushupController.hh"
#include "VirtualInput.hh"

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

        // Diable all motor drives
        for ( i = 0 ; i < 6 ; i++) 
          hw.driveEnable ( i, false );

        MMShutdown();
        MMPowerOff();

      }

    }
} umod;

int main ( void ) {

#ifdef _MICHIGAN_
#ifdef _QNX4_
  //  setprio( getpid(), 23 );
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
  MMReadConfigFile( "sup.rc" );

  MMChooseHardware ( & hw );

  Supervisor supervisor;
  PushupController puc;

  // Create and add all the standard modules
  RHexAddStdModules();

  // Add the pushup controller and supervisor
  MMAddModule ( &umod, 1, 0, USER_CONTROLLERS );
  MMAddModule ( &puc, 1, 0, USER_CONTROLLERS );
  MMAddModule ( &supervisor, 1, 0, USER_CONTROLLERS );

  // Activate the supervisor modules
  MMActivateModule ( &umod );
  MMActivateModule ( &supervisor );

#ifdef _LINUX_

  // In Linux, The logger module is mainly for debugging purposes.
  // Under QNX, the Supervisor handles the turning on and off of the logger
  MMActivateModule ( rhexlogger );

#endif

  MMMainLoop();
  MMShutdown();
  return 0;

}







