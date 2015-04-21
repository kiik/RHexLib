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
 * $Id: Supervisor.hh,v 1.1 2001/08/06 17:01:44 ulucs Exp $
 *
 ********************************************************************/

#ifndef _SUPERVISOR_HH
#define _SUPERVISOR_HH

#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "StdModules.hh"
#include "StandMachine.hh"
#include "SitMachine.hh"
#include "CalibMachine.hh"
#include "RemoteControl.hh"
#include "RHexWalker.hh"
#include "ExtProfiler.hh"

#define ACCEL_TIME 2.5

class Supervisor : public StateMachine {

  public:

    Supervisor ( void );
    ~Supervisor ( void );
    void init ( void );
    void activate ( void );
    void deactivate ( void );
    void setDirection ( void );  
 
  private:

    // events
    EventObject ( StartCommand ) * startCommand;
    EventObject ( CalFail ) * calFail;
    EventObject ( CalSuccess ) * calSuccess;
    EventObject ( DoneStanding ) * doneStanding;
    EventObject ( AccWalkCommand ) * accWalkCommand;
    EventObject ( WalkCommand ) * walkCommand;
    EventObject ( UpToSpeed ) * upToSpeed;
    EventObject ( NoCommand ) * noCommand;
    EventObject ( StopCommand ) * stopCommand;
    EventObject ( DoneDecel ) * doneDecel;

    // states
    StateObject ( UnCalibrated ) * unCalibrated;
    StateObject ( Calibrating ) * calibrating;
    StateObject ( Standing ) * standing;
    StateObject ( Ready ) * ready;
    StateObject ( Accelerating ) * accelerating;
    StateObject ( Walking ) * walking;
    StateObject ( Decelerating ) * decelerating;

    // data
    StandMachine     * standMachine;
    RHexWalker       * walkMachine;
    CalibMachine     * calibMachine[6];
    Hardware         * hw;
    RemoteControl    * rc;
    ExtProfiler      accelProfile;
    double           mark;
    int              direction;

};

#endif
