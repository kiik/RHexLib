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
 * $Id: PushupController.hh,v 1.2 2001/08/14 03:08:52 ulucs Exp $
 *
 * The PushupController Module.
 *
 * This file contains the definition of the PushupController class 
 * and all the associated definitions. The member function definitions
 * are in PushupController.cc
 *
 * Created       : Uluc Saranli, 06/01/2000
 * Last Modified : Uluc Saranli, 07/13/2001
 *
 ********************************************************************/

#ifndef _PUSHUPCONTROLLER_HH
#define _PUSHUPCONTROLLER_HH

// Local includes
#include "ModuleManager.hh"
#include "PositionControl.hh"
#include "Profiler.hh"

// Module specific constants ---------------------------------------

// The PushupController class -----------------------------------------
class PushupController : public Module {

public:
  PushupController ( void )
    : Module( "pushupcontroller", 0, true, false ) { };

  // Module base class interface
  void  init ( void );
  void  uninit ( void ) { };
  void  activate ( void );
  void  deactivate ( void );
  void  update ( void );

private:

  PositionControl *control[6];
  MotorGains_t     oldGains[6];
  Profiler         legProfiler;

  double lowerTime;  // Time in microseconds to lower the robot
  double chillTime;  // Time in microseconds to chill, sitting down
  double standTime;  // Time in microseconds to raise the robot up
  double waitTime;   // Time in microseconds to wait standing up.

  // The current controller phase.
  //
  // 0 : Starting to lower the robot
  // 1 : Waiting for the robot to lower
  // 2 : Wait for a while in the sitting position
  // 3 : Starting to standup
  // 4 : Waiting for the robot to standup
  // 5 : Wait for a while in the standing position
  int phase;

  // Time mark to record beginning of various phases.
  double mark;

  // Utility functions
  void trackCurrentProfile( void );
};

#endif
