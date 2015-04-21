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
 * $Id: calib_test.cc,v 1.2 2001/08/08 22:55:42 mcmordie Exp $
 *
 * Example program to test the CalibMachine
 *
 ********************************************************************/

#include <stdio.h>
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif

#include "sysutil.hh"
#include "types.hh"
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "CalibMachine.hh"
#include "PositionControl.hh"
#include "EncoderReader.hh"

#ifdef _QNX4_

#ifdef _MICHIGAN_
#include "MichiganHW.hh"
MichiganHW hw;
#endif

#ifdef _MCGILL_
#include "McGillHW.hh"
McGillHW hw;
#endif
#endif

#ifdef _LINUX_
#include "SimSectHW.hh"
SimSectHW       hw;
#endif

class SafetyModule : public Module {
public:
  SafetyModule ( void )
    : Module( "safety", 0, false, false ) { };

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { calibState = 0; };
  void  deactivate ( void ) { };
  void  update ( void ) {
    int i;

    if ( kbhit() ) {
      MMMessage( "User interrupt: Shutting down!\n" );

      MMPowerOff( );
    }

    if ( calibState == 0 ) {
      calibState = 1;

      for ( i = 0; i < 6; i++ )
        if ( calibmachine[i]->getStatus() != CalibMachine::SUCCESS )
          calibState = 0;
    } else if ( calibState == 1 ) {

      MotorTarget_t target;
      MotorGains_t gains;

      gains.kp = 2.0 * 9.606;
      gains.kd = 2.0 * 0.72045;
      target.pos = M_PI;
      target.vel = 0.0;
      target.acc = 0.0;

      for ( i = 0; i < 6; i++ )
        MMDeactivateModule( calibmachine[i] );

      for ( i = 0; i < 6; i++ ) {
        hw.driveEnable( i, true );
        MMGrabModule( poscontrol[i], this );
        poscontrol[i]->setGains ( &gains );
        poscontrol[i]->setTarget ( &target );
      }

      calibState = 2;
    }

  }

private:

  int calibState;

};


int main ( void ) {

  int i;
  SafetyModule      sf;

#ifdef _QNX4_
  //  setprio( getpid(), 23 );
#endif

  MMReadConfigFile( "calib_test.rc" );
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

  MMChooseHardware( &hw );

  RHexAddStdModules();

  // Adding modules
  MMAddModule( &sf, 10, 0, OTHER_MODULES );

  // Module activation
  MMActivateModule( &sf );

  for ( i = 0; i < 6; i++ )
    MMActivateModule( calibmachine[i] );

  MMMainLoop();

  MMShutdown();

  return 0;

}
