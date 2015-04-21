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
 * $Id: StateMachine.hh,v 1.3 2001/07/24 02:06:29 ulucs Exp $
 *
 * State Machine Header File
 *
 * This file contains all datatypes, class and function prototypes for 
 * the State Machine Module. The functions declared in this file are
 * implimented in StateMachine.cc
 *
 * Created       : Eric Klavins, 11/03/2000
 * Last Modified : Eric Klavins, 11/11/2000
 *
 ********************************************************************/

#ifndef _STATEMACHINE_HH
#define _STATEMACHINE_HH

// Local includes
#include "ModuleManager.hh"

class StateMachine;

class Event { 

  public:

    Event ( StateMachine * p, const char n[] = NULL ) { owner = p; name = n; };
    virtual bool check ( void ) = 0;

    const char * getName( void ) { return name; };

    StateMachine * owner;  // Pointer to the owner state machine for this event

  private:

    const char *name;      // Pointer to the name of this event
};

class State;

class Arc {

  friend class State;
  friend class StateMachine;

  public:
 
    Arc ( State * s, Event * e ) { target = s; ev = e; };
    State * getTarget ( void ) { return target; };

  private:

    Event * ev;
    State * target;

};

#define MAXARCS 20

class State {

  friend class StateMachine;

  public:

    State ( StateMachine * p, const char n[] = NULL ) { 
      owner = p; numarcs = 0; name = n; 
    };

    bool addArc ( Arc * a ) { 
      if ( numarcs < MAXARCS ) {
        arclist[numarcs++] = a;
        return true;
      } else return false;
    };

    Arc * getActiveArc ( void );

    virtual void entry ( void ) = 0;
    virtual void during ( void ) = 0;
    virtual void exit ( void ) = 0;
       
    const char * getName( void ) { return name; };

    StateMachine * owner;  // Pointer to the owner state machine for this event

  private:

    int numarcs;
    Arc * arclist[MAXARCS]; // god I am so lazy

    const char *name;       // Pointer to the name of this event
};

class StateMachine : public Module {

  public:

    StateMachine( char *n ) : Module ( n, 0, true, false ) 
     { initial = NULL; };
    StateMachine( char *n, State *i ) : Module ( n, 0, true, false ) 
     { initial = i; };
    StateMachine( char *n, uint index ) : Module ( n, index, true, false )
     { initial = NULL; };
    StateMachine( char *n, uint index, State *i ) : Module ( n, index, true, false ) 
     { initial = i; };

    void initialize ( State * i ) { initial = i; };
    State *getCurState( void ) { return current; };

    // module methods
    void init ( void ) { current = initial; }
    void uninit ( void ) {};
    void activate ( void ) { current = initial; current->entry(); };
    void deactivate ( void ) {};
    void update ( void );

  private:

    // pointers to the initial state and the current state
    State * initial, * current;

};

// Some macros to save you some typing

#define Transition( _from_, _event_, _to_ )           \
  _from_->addArc ( new Arc ( _to_, _event_ ) )

// useage: EventObject ( MyEvent ) * myEvent;
#define EventObject( _name_ )                         \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public Event {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL  )               \
      : Event ( m, name ) {};                         \
    bool check ( void );                              \
                                                      \
};                                                    \
_name_

// useage: StateObject ( MyState ) * myState;
#define StateObject( _name_ )                         \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void );                              \
    void during ( void );                             \
    void exit ( void );                               \
                                                      \
};                                                    \
_name_

//
// These save typing and make the code easier to read. Each
// is of the form StateObjectXYZ. If X is "E", then you need to
// define the entry function somewhere, if it is "e", then you
// do not. The meanings Y and Z are similar.
//

#define StateObjectEDX( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void );                              \
    void during ( void );                             \
    void exit ( void );                               \
                                                      \
};                                                    \
_name_

#define StateObjectEDx( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void );                              \
    void during ( void );                             \
    void exit ( void ) {}                             \
                                                      \
};                                                    \
_name_

#define StateObjectEdX( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void );                              \
    void during ( void ) {}                           \
    void exit ( void );                               \
                                                      \
};                                                    \
_name_

#define StateObjectEdx( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void );                              \
    void during ( void ) {}                           \
    void exit ( void ) {}                             \
                                                      \
};                                                    \
_name_

#define StateObjecteDX( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void ) {}                            \
    void during ( void );                             \
    void exit ( void );                               \
                                                      \
};                                                    \
_name_

#define StateObjecteDx( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void ) {}                            \
    void during ( void );                             \
    void exit ( void ) {}                             \
                                                      \
};                                                    \
_name_

#define StateObjectedX( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void ) {}                            \
    void during ( void ) {}                           \
    void exit ( void );                               \
                                                      \
};                                                    \
_name_

#define StateObjectedx( _name_ )                      \
class _name_;                                         \
friend class _name_;                                  \
class _name_ : public State {                         \
                                                      \
  public:                                             \
                                                      \
    _name_ ( StateMachine * m,                        \
             const char *name = NULL )                \
      : State ( m, name ) {};                         \
    void entry ( void ) {}                            \
    void during ( void ) {}                           \
    void exit ( void ) {}                             \
                                                      \
};                                                    \
_name_

#endif








