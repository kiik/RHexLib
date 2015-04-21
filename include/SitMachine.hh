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
 * $Id: SitMachine.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/04/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _SITMACHINE_HH
#define _SITMACHINE_HH

// System includes
#include <stdio.h>
#include <math.h>
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "MotorControl.hh"
#include "EncoderReader.hh"
#include "Profiler.hh"

// Module specific constants ---------------------------------------

// The SitMachine class --------------------------------------------
class SitMachine : public StateMachine {

public:

  SitMachine ( void );
  ~SitMachine ( void );

  void init ( void );
  void activate ( void );
  void deactivate ( void );

  bool isDone ( void ) { return done; }

private:

  // events
  EventObject( DoneSitting ) * doneSitting;
 
  // states
  StateObject( Sit ) * sit;                // sitting
  StateObjecteDx( SitDone ) * sitDone;     // finished sitting

  // data
  MotorControl  * control[6];
  EncoderReader * enc[6];
  Profiler prof[6];

  double   timeSit;

  double   mark;
  bool     done;

  // Utility functions
  void gotoAngle( uint leg, double time, float angle );
  void trackCurrentProfile ( void );

};

#endif







