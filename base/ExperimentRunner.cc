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
 * $Id: ExperimentRunner.cc,v 1.3 2001/07/19 18:51:30 eklav Exp $
 *
 * Created       : Uluc Saranli, 01/21/2001
 * Last Modified : Eric Klavins, 07/19/2001
 *
 ********************************************************************/

#include <math.h>
#include <stdio.h>
#include <time.h>
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "ExperimentRunner.hh"

#define EXP_DEBUG_MSG( str )  // MMMessage( str )

// ---------------------------------------------------------------------------
// define the state methods --------------------------------------------------
// ---------------------------------------------------------------------------

//******************
// IdleState
//******************
void ExperimentRunner::IdleState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::IdleState::entry\n" );

  OWNER->idleEntry( OWNER->expNumber );

  OWNER->status = IDLE; 
}
void ExperimentRunner::IdleState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::IdleState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->skipEv->check() ) {
    // Skipping the current experiment
    OWNER->nextExperiment( true );
  }
}

//******************
// SetupState
// *****************
void ExperimentRunner::SetupState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::SetupState::entry\n" );

  OWNER->setupEntry( OWNER->expNumber );

  OWNER->status = SETUP; 
} 
void ExperimentRunner::SetupState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::SetupState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {
    MMMessage( "ExperimentRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );
  } 
}

//******************
// ExperimentState
//******************
void ExperimentRunner::ExperimentState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::ExperimentState::entry\n" );

  OWNER->experimentEntry( OWNER->expNumber );

  // Grab and Reset the logger for new data
  if ( OWNER->logFlag ) {
    MMGrabModule( OWNER->logger, OWNER );
    OWNER->logger->reset();
  }

  OWNER->status = RUNNING; 
}
void ExperimentRunner::ExperimentState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::ExperimentState::exit\n" );

  // Done with the experiment. Stop the logger
  if ( OWNER->logFlag )
    MMReleaseModule( OWNER->logger, OWNER );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {

    MMMessage( "ExperimentRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );

  } else
    OWNER->experimentExit( OWNER->expNumber );
}

//******************
// WrapupState
//******************
void ExperimentRunner::WrapupState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::WrapupState::entry\n" );

  OWNER->status = WRAPUP; 
}
void ExperimentRunner::WrapupState::exit ( void ) { 

  EXP_DEBUG_MSG( "ExperimentRunner::WrapupState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {

    MMMessage( "ExperimentRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );

  } else {

    // Proceed with the next experiment
    OWNER->wrapupExit( OWNER->expNumber );
    OWNER->nextExperiment( false );
  }
}

//******************
// FinishedState
//******************
void ExperimentRunner::FinishedState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExperimentRunner::FinishedState::entry\n" );

  OWNER->status = DONE; 
}

// ---------------------------------------------------------------------------
// ExperimentRunner methods --------------------------------------------------
// ---------------------------------------------------------------------------

ExperimentRunner::ExperimentRunner( char *name, uint dataLen ) 
  : StateMachine( name ) {

  // allocate events
  setupBeginEv = new SetupBeginEv( this );
  experimentBeginEv = new ExperimentBeginEv( this );
  experimentEndEv = new ExperimentEndEv( this );
  wrapupEndEv = new WrapupEndEv( this );
  terminateEv = new TerminateEv( this );
  abortEv = new AbortEv( this );
  skipEv = new SkipEv( this );
  repeatEv = new RepeatEv( this );

  // allocate states
  idleState = new IdleState( this );
  setupState = new SetupState( this );
  experimentState = new ExperimentState( this );
  wrapupState = new WrapupState( this );
  finishedState = new FinishedState( this );

  // ************************************************************
  // add transitions ( this defines the structure of the machine )
  // 
  //          From State        Event              To State
  //-----------------------------------------------------------
  Transition ( idleState,       setupBeginEv,      setupState );
  Transition ( idleState,       skipEv,            idleState );
  Transition ( idleState,       abortEv,           finishedState );

  Transition ( setupState,      experimentBeginEv, experimentState );
  Transition ( setupState,      abortEv,           finishedState );
  Transition ( setupState,      repeatEv,          idleState );
  Transition ( experimentState, experimentEndEv,   wrapupState );
  Transition ( experimentState, abortEv,           finishedState );
  Transition ( wrapupState,     wrapupEndEv,       idleState );
  Transition ( wrapupState,     abortEv,           finishedState );

  Transition ( idleState,       terminateEv,       finishedState );

  // define state machine
  initialize ( idleState );

  // Create the logger
  logger = new ExperimentLogger( dataLen, this );

  // Default configuration
  strcpy ( baseName, "exp" );

  logPeriod = 5;
  logFlag = false;

  // Start with no log messages
  firstLog = lastLog = NULL;

  // Initialize the dynamic data
  expNumber = 0;
  status = IDLE;
}

ExperimentRunner::~ExperimentRunner( ) {

  // deallocate events
  if ( setupBeginEv ) delete setupBeginEv;
  if ( experimentBeginEv ) delete experimentBeginEv;
  if ( experimentEndEv ) delete experimentEndEv;
  if ( wrapupEndEv ) delete wrapupEndEv;
  if ( terminateEv ) delete terminateEv;
  if ( abortEv ) delete abortEv;
  if ( skipEv ) delete skipEv;
  if ( repeatEv ) delete repeatEv;

  // deallocate states
  if ( idleState ) delete idleState;
  if ( setupState ) delete setupState;
  if ( experimentState ) delete experimentState;
  if ( wrapupState ) delete wrapupState;
  if ( finishedState ) delete finishedState;

  // Deallocate the experiment logger
  if ( logger ) delete logger;

}

void ExperimentRunner::configure( char *n, bool logging, MM_STEP p ) { 

  strcpy ( baseName, n );

  logPeriod = p; 
  
  if ( getState() == Module::ACTIVE ) {

    if ( logFlag && !logging ) {
      // Disabling logging
      MMReleaseModule( logger, this );
      MMDeactivateModule( logger );
    } else if ( !logFlag && logging ) {
      // Enabling logging
      MMGrabModule( logger, this );
    }
  }

  logFlag = logging; 

};

void ExperimentRunner::logMessage( char *msg ) {
  logEntry *newEntry;

  newEntry = new logEntry;

  // Create a new log entry with a copy of the message
  newEntry->msg = new char[ strlen( msg ) + 1 ];
  strcpy( newEntry->msg, msg );
  newEntry->next = NULL;

  // Add the log message to the list
  if ( firstLog == NULL ) {
    firstLog = lastLog = newEntry;
  } else {
    lastLog->next = newEntry;
    lastLog = newEntry;
  }
}

void ExperimentRunner::init ( void ) {

  StateMachine::init();

  logger->reset();

  MMAddModule( logger, logPeriod, 0, LOGGING_MODULES );

  // Record the starting time of the batch for generating filenames
  startTime = time( NULL );

}

void ExperimentRunner::activate ( void ) {

  StateMachine::activate();

}

void ExperimentRunner::deactivate ( void ) {

  StateMachine::deactivate();

}

// ExperimentRunner::dumpLog : Appends the current log messages to the log file */
void ExperimentRunner::dumpLog( char *logname, char *paramname, char *expname ) {
  FILE *logFile, *paramFile;
  char msg[2048], *timestr;
  logEntry *curEntry;
  time_t curTime;

  if ( ( logFile = fopen( logname, "a" ) ) == NULL ) {
    sprintf( msg, "Cannot open the log file \"%s\"!\n", logname );
    MMWarning( "ExperimentRunner::dumpLog", msg );
    return;
  }
  
  if ( ( paramFile = fopen( paramname, "a" ) ) == NULL ) {
    sprintf( msg, "Cannot open the parameter dump file \"%s\"!\n", paramname );
    MMWarning( "ExperimentRunner::dumpLog", msg );
    return;
  }
  
  // Get the calendar time and print it to the log file.
  curTime = time( NULL );
  timestr = ctime( &curTime );
  timestr[ strlen(timestr) - 1 ] = 0; // delete the trailing newline on the string

  fprintf( logFile,  "time: %s, exp#%i, file: %s\n", timestr, expNumber, expname );

  // Dump all the log messages to the file, assuming that the first message
  // gives a comma sepaerated parameter list
  if ( firstLog != NULL ) {
    fprintf( logFile, " params:%s\n", firstLog->msg );
    fprintf( paramFile, "%s,%i,%s\n", expname, expNumber, firstLog->msg );
    curEntry = firstLog->next;

    while ( curEntry != NULL ) {
      fprintf( logFile, "%s\n", curEntry->msg );
      curEntry = curEntry->next;
    }
  }

  fprintf( logFile, "\n" );

  fclose( logFile );
  fclose( paramFile );

  // Clear the current log messages
  clearLog();
}

// ExperimentRunner::clearLog : Clears the current list of log messages */
void ExperimentRunner::clearLog( ) {
  logEntry *cur = firstLog, *next;

  while ( cur != NULL ) {
    if ( cur->msg ) delete[] cur->msg;
    next = cur->next;
    delete cur;
    cur = next;
  }

  firstLog = lastLog = NULL;

}

// ExperimentRunner::nextExperiment :
// Proceeds with the next experiment after dumping the DataLogger data and
// appending the log messages on the experiment log file
void ExperimentRunner::nextExperiment( bool skip ) {
  char filename[128];
  char logname[128];
  char paramname[128];
  char msg[128];
  struct tm *timeDetail;

  if ( logFlag ) {
    // Construct the filename for the data output
    timeDetail = localtime( &startTime );

    sprintf( filename, "%s%02i%02i%02i.%02i:%02i.%04i.data", 
             baseName, 
             timeDetail->tm_year % 100,
             timeDetail->tm_mon + 1,
             timeDetail->tm_mday,
             timeDetail->tm_hour,
             timeDetail->tm_min, 
             expNumber );

    if ( !skip ) {
      sprintf( msg, "Experiment %i completed. Saving data to file \"%s\"...", 
               expNumber, filename );
      MMMessage( msg );

      logger->exportToFile( filename, DL_ASCII );

    } else {
      sprintf( msg, "Skipping experiment %i, Data not saved!...", expNumber );
      MMMessage( msg );
      logMessage( "Experiment skipped!\n" );
    }

    // Log the experiment details to the log file to a param file 
    // in Matlab loadable format
    sprintf( logname, "%s%02i%02i%02i.log", 
             baseName,
             timeDetail->tm_year % 100,
             timeDetail->tm_mon + 1,
             timeDetail->tm_mday);
    sprintf( paramname, "%s%02i%02i%02i.param", 
             baseName,
             timeDetail->tm_year % 100,
             timeDetail->tm_mon + 1,
             timeDetail->tm_mday);
    dumpLog( logname, paramname, filename );

  } else {
    if ( !skip )
      sprintf( msg, "Experiment %i completed. Cleaning up...", expNumber );
    else 
      sprintf( msg, "Experiment %i skipped. Cleaning up...", expNumber );

    MMMessage( msg );
  }

  // Compute new experiment number
  expNumber++;

  MMMessage( "done.\n" );
}

