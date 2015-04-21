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

/*
 *
 * RHEXLIB 
 *
 * ToyMachine.cc
 *
 * This file and all of RHEXLIB are copyright 2000/2001 by the University
 * of Michigan. Copying and/or distributing require the expressed
 * written consent of the authors or maintainers of this code.
 *
 */

#include "ToyMachine.hh"

#define OWNER ( ( ToyMachine * ) owner )

// Events ------------------------------------------------------------
bool ToyMachine::EventOne::check ( void ) { 

  return bool( MMReadTime() > OWNER->mark + 1.0 );

}

// States ------------------------------------------------------------
void ToyMachine::StateOne::entry ( void ) {

  OWNER->mark = MMReadTime();
  printf ( "State One Entered at Time %f\n", OWNER->mark );

}
void ToyMachine::StateOne::during ( void ) {}
void ToyMachine::StateOne::exit ( void ) {
 
  printf ( "State One Exited at Time %f\n", MMReadTime() );

}

void ToyMachine::StateTwo::entry ( void ) {

  printf ( "State Two Entered at Time %f\n", MMReadClock() );

}
void ToyMachine::StateTwo::during ( void ) {}
void ToyMachine::StateTwo::exit ( void ) {}

ToyMachine::ToyMachine ( void ) : StateMachine ( "toymachine" ) {

  // allocate events
  eventOne = new EventOne ( this );

  // allocate states
  stateOne = new StateOne ( this );
  stateTwo = new StateTwo ( this );

  // transitions ( in the form < From, Event, To > )
  Transition ( stateOne, eventOne, stateTwo );

  // the initial state
  initialize ( stateOne );

} 

ToyMachine::~ToyMachine ( void ) {

  if ( eventOne ) delete ( eventOne );
  if ( stateOne ) delete ( stateOne );
  if ( stateTwo ) delete ( stateTwo );

}

void ToyMachine::init ( void ) {

  StateMachine::init();

}

void ToyMachine::activate ( void ) {

  StateMachine::activate();

}

void ToyMachine::deactivate ( void ) {

  StateMachine::deactivate();

}
