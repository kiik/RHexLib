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
 * $Id: ExpRunner.hh,v 1.4 2001/08/12 16:01:50 ulucs Exp $
 *
 * Improved Experiment Runner facility
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Robert Peters, 08/09/2001
 *
 ********************************************************************/

#ifndef _EXPRUNNER_HH
#define _EXPRUNNER_HH

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <fstream>

#include <iostream>

#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "DataLogger.hh"
#include "RHexLogger.hh"
#include "Strings.hh"
#include "Floats.hh"

using namespace std;

// Module specific constants ---------------------------------------

// Data types ------------------------------------------------------

// The ExpRunner class ---------------------------------------------
class ExpRunner : public StateMachine {

public:

  // Structure to hold a single log entry
  struct _logEntry {

    char *msg;
    struct _logEntry *next;

  };
  typedef struct _logEntry logEntry;

  ExpRunner( char *name, uint dataLen );
  ~ExpRunner( void );

  typedef enum { INIT, IDLE, SETUP, RUNNING, WRAPUP, DONE } RunnerStatus;

  void         setLogFlag( bool logging );
  //  void         configure( char *n, bool logging, MM_STEP p );
  int          getExpNumber( void ) { return expNumber; };
  bool         isDone( void ) { return ( status == DONE ) ? true : false; };
  RunnerStatus getStatus( void ) { return status; };

  // Used to put a string in the log file
  void logMessage( char *msg );

  // Interface to the particular experiment definition
  virtual bool setupBeginCheck( int num ) = 0;
  virtual bool experimentBeginCheck( int num ) = 0;
  virtual bool experimentEndCheck( int num ) = 0;
  virtual bool wrapupEndCheck( int num ) = 0;

  // Event to abort the batch
  virtual bool abortCheck( int num ) { num = 0; return false; };
  // Event to skip the current experiment 
  virtual bool skipCheck( int num ) { num = 0; return false; };
  // Event to terminate the batch
  virtual bool terminateCheck( int num ) = 0;
  // Event to repeat the current experiment 
  virtual bool repeatCheck( int num ) { num = 0; return false; };

  virtual void initEntry( int num ) = 0;
  virtual void idleEntry( int num ) { num = 0; };
  virtual void setupEntry( int num ) = 0;
  virtual void setupDuring( int num ) { num = 0; };
  virtual void experimentEntry( int num ) = 0;
  virtual void experimentDuring( int num ) { num = 0; };
  virtual void experimentExit( int num ) = 0;
  virtual void wrapupDuring( int num ) { num = 0; };
  virtual void wrapupExit( int num ) = 0;

  virtual void handleAbort( int num ) = 0;
  virtual void handleRepeat( int num ) = 0;

  void setParameters( Strings names );
  void setParameterValues( Floats values );


  // Function to fill in the data
  //  virtual void fillRecord( float *f ) = 0;

  // Standard StateMachine module functions
  void init ( void );
  void activate ( void );
  void deactivate ( void );

private:

  // events
  EventObject ( SetupBeginEv ) * setupBeginEv;
  EventObject ( ExperimentBeginEv ) * experimentBeginEv;
  EventObject ( ExperimentEndEv ) * experimentEndEv;
  EventObject ( WrapupEndEv ) * wrapupEndEv;
  EventObject ( TerminateEv ) * terminateEv;
  EventObject ( AbortEv ) * abortEv;
  EventObject ( SkipEv ) * skipEv;
  EventObject ( RepeatEv ) * repeatEv;

  // states
  StateObjectEdX ( InitState ) * initState;
  StateObjectEdX ( IdleState ) * idleState;
  StateObjectEDX ( SetupState ) * setupState;
  StateObjectEDX ( ExperimentState ) * experimentState;
  StateObjectEdX ( WrapupState ) * wrapupState;
  StateObjectEdx ( FinishedState ) * finishedState;

  // Configuration
  char *baseName;       // Base filename for the output
  MM_STEP logPeriod;    // Module period for the logger
  bool logFlag;
  bool setParamsCalled;

  // The logger object
  RHexLogger  *logger;    

  // Logging of experiment information 
  logEntry *firstLog;
  logEntry *lastLog;
  Strings loggedNames;
  Strings parameterNames;
  Floats parameterValues;
  ofstream outstream;
  char thisfilename[128];

  // Starting time of the batch
  time_t startTime;

  // Dynamic data
  int expNumber;
  RunnerStatus status;

  void dumpLog( void );
  void clearLog( void );
  void nextExperiment( bool skip );

};

// ---------------------------------------------------------------------------
// The event check methods ---------------------------------------------------
// ---------------------------------------------------------------------------


// This is defined to avoid conficts with other OWNER definitions
#define EXPRUNNER_OWNER ( ( ExpRunner * ) owner )

inline bool ExpRunner::SetupBeginEv::check ( void ) {
  return bool(!EXPRUNNER_OWNER->terminateCheck( EXPRUNNER_OWNER->expNumber ) 
              && EXPRUNNER_OWNER->setupBeginCheck(EXPRUNNER_OWNER->expNumber));
}

inline bool ExpRunner::ExperimentBeginEv::check ( void ) {
  return EXPRUNNER_OWNER->experimentBeginCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::ExperimentEndEv::check ( void ) {
  return EXPRUNNER_OWNER->experimentEndCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::WrapupEndEv::check ( void ) {
  return EXPRUNNER_OWNER->wrapupEndCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::TerminateEv::check ( void ) {
  return EXPRUNNER_OWNER->terminateCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::SkipEv::check ( void ) {
  return EXPRUNNER_OWNER->skipCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::AbortEv::check ( void ) {
  return EXPRUNNER_OWNER->abortCheck( EXPRUNNER_OWNER->expNumber );
}

inline bool ExpRunner::RepeatEv::check ( void ) {
  return EXPRUNNER_OWNER->repeatCheck( EXPRUNNER_OWNER->expNumber );
}


// ---------------------------------------------------------------------------
// The state during methods --------------------------------------------------
// ---------------------------------------------------------------------------

inline void ExpRunner::SetupState::during ( void ) { 
  EXPRUNNER_OWNER->setupDuring( EXPRUNNER_OWNER->expNumber );
}

inline void ExpRunner::ExperimentState::during ( void ) { 
  EXPRUNNER_OWNER->experimentDuring( EXPRUNNER_OWNER->expNumber );
}

#endif
