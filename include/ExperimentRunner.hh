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
 * $Id: ExperimentRunner.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Uluc Saranli, 01/20/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _EXPERIMENTRUNNER_HH
#define _EXPERIMENTRUNNER_HH

// System includes
#include <stdio.h>
#include <math.h>
#include <time.h>

// Local includes
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "DataLogger.hh"

// Module specific constants ---------------------------------------

// Data types ------------------------------------------------------

// Structure to hold a single log entry
struct _logEntry {

  char *msg;

  struct _logEntry *next;

};
typedef struct _logEntry logEntry;

// The ExperimentLogger class -----------------------------------------
class ExperimentRunner;

class ExperimentLogger : public DataLogger {

public:

  ExperimentLogger ( uint dataLen, ExperimentRunner *er ) 
    : DataLogger ( dataLen, "explogger" ) { runner = er; };

  void fillRecord ( float * f );   // Defined as inline, below

private:

  ExperimentRunner *runner;

};

// The ExperimentRunner class -----------------------------------------
class ExperimentRunner : public StateMachine {

public:

  ExperimentRunner( char *name, uint dataLen );
  ~ExperimentRunner( void );

  typedef enum { IDLE, SETUP, RUNNING, WRAPUP, DONE } RunnerStatus;

  void         configure( char *n, bool logging, MM_STEP p );
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
  // Event to repeat the setup process from the begining
  virtual bool repeatCheck( int num ) { num = 0; return false; };
  // Event to terminate the batch
  virtual bool terminateCheck( int num ) = 0;

  virtual void idleEntry( int num ) { num = 0; };
  virtual void setupEntry( int num ) = 0;
  virtual void setupDuring( int num ) { num = 0; };
  virtual void experimentEntry( int num ) = 0;
  virtual void experimentDuring( int num ) { num = 0; };
  virtual void experimentExit( int num ) = 0;
  virtual void wrapupDuring( int num ) { num = 0; };
  virtual void wrapupExit( int num ) = 0;

  virtual void handleAbort( int num ) = 0;

  // Function to fill in the data
  virtual void fillRecord( float *f ) = 0;

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
  StateObjectEdX ( IdleState ) * idleState;
  StateObjectEDX ( SetupState ) * setupState;
  StateObjectEDX ( ExperimentState ) * experimentState;
  StateObjectEdX ( WrapupState ) * wrapupState;
  StateObjectEdx ( FinishedState ) * finishedState;

  // Configuration

  char baseName[256];   // Base filename for the output
  MM_STEP logPeriod;    // Module period for the logger
  bool logFlag;

  // The logger object
  ExperimentLogger  *logger;    

  // Logging of experiment information
  logEntry *firstLog;
  logEntry *lastLog;

  // Starting time of the batch
  time_t startTime;

  // Dynamic data
  int expNumber;
  RunnerStatus status;

  void dumpLog( char *filename, char *paramname, char *expname );
  void clearLog( void );
  void nextExperiment( bool skip );
};

inline void ExperimentLogger::fillRecord( float *f ) { 
  runner->fillRecord( f );
 }

// ----------------------------------------------------------------------------
// The event check methods ----------------------------------------------------
// ----------------------------------------------------------------------------

#define OWNER ( ( ExperimentRunner * ) owner )

inline bool ExperimentRunner::SetupBeginEv::check ( void ) {
  return bool ( !OWNER->terminateCheck( OWNER->expNumber )
                && OWNER->setupBeginCheck( OWNER->expNumber ) );
}

inline bool ExperimentRunner::ExperimentBeginEv::check ( void ) {
  return OWNER->experimentBeginCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::ExperimentEndEv::check ( void ) {
  return OWNER->experimentEndCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::WrapupEndEv::check ( void ) {
  return OWNER->wrapupEndCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::TerminateEv::check ( void ) {
  return OWNER->terminateCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::SkipEv::check ( void ) {
  return OWNER->skipCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::RepeatEv::check ( void ) {
  return OWNER->repeatCheck( OWNER->expNumber );
}

inline bool ExperimentRunner::AbortEv::check ( void ) {
  return OWNER->abortCheck( OWNER->expNumber );
}

// ---------------------------------------------------------------------------
// The state during methods --------------------------------------------------
// ---------------------------------------------------------------------------

inline void ExperimentRunner::SetupState::during ( void ) { 
  OWNER->setupDuring( OWNER->expNumber );
}

inline void ExperimentRunner::ExperimentState::during ( void ) { 
  OWNER->experimentDuring( OWNER->expNumber );
}

#endif
