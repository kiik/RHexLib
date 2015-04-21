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
 * $Id: StartupMachine.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Startup controller
 *
 * This file contains the definition of the StartupMachine module 
 * and all the associated definitions. The member function definitions
 * are in StartupMachine.cc
 *
 * The StartupMachine module uses CalibMachine and StandMachine to
 * first calibrate RHex and then stand up. These actions immediately 
 * follow each other without any user intervention. The StandMachine
 * remains active until StartupMachine is deactivated.
 *
 * Created       : Uluc Saranli, 01/22/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _STARTUPMACHINE_HH
#define _STARTUPMACHINE_HH

// System includes
#include <stdio.h>
#include <math.h>

// Local includes
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "StandMachine.hh"
#include "CalibMachine.hh"

// Module specific constants ---------------------------------------

// The StartupMachine class ----------------------------------------
class StartupMachine : public StateMachine {

public:

  typedef enum { CALIBRATING, STANDING, FAILURE, SUCCESS } MachineStatus;

  StartupMachine( void );
  ~StartupMachine( void );

  void init( void );
  void deactivate( void );

  void setLegOffset( float offset ) { sm->setLegOffset( offset ); };
 
  void setCalibMode( CalibMachine::CalibMode m ) { calibMode = m; };

  MachineStatus getStatus( void ) { return status; };
  
private:

  // events
  EventObject ( CalibSuccess ) * calibSuccess;    // calibration worked
  EventObject ( CalibFail ) * calibFail;          // calibration failed
  EventObject ( StandDoneEv ) * standDoneEv;      // stand machine says done

  // states
  StateObjectEdX ( Calibrating ) * calibrating;   // running the calibration machine
  StateObjectEdX ( Standing ) * standing;         // running the stand machine
  StateObjectEdx ( DoneStanding ) * doneStanding; // finished standing
  StateObjectEdx ( FailedState ) * failedState;   // failed to calibrate

  // Modules used by the StartupMachine
  StandMachine * sm;
  CalibMachine * cm[6];
  Hardware * hw;

  CalibMachine::CalibMode calibMode;

  MachineStatus status;
};

#endif
