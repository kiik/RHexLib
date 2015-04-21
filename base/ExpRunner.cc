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
 * $Id: ExpRunner.cc,v 1.5 2001/08/14 03:08:26 ulucs Exp $
 *
 * Improved Experiment Runner facility
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Robert Peters, 08/09/2001
 *
 ********************************************************************/

#include <math.h>
#include <stdio.h>
#include <time.h>
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "ExpRunner.hh"

#define OWNER ( ( ExpRunner * ) owner )

#define EXP_DEBUG_MSG( str ) // MMMessage( str )

// ---------------------------------------------------------------------------
// define the state methods --------------------------------------------------
// ---------------------------------------------------------------------------

//******************
// InitState
//******************
void ExpRunner::InitState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::InitState::entry\n" );


  char basefilename[128];
  char temp[128], msg[128];
  char ch;
  int i = 0;
  int j = 0;
  ifstream instream;
  ofstream outstream;
  time_t seconds = time( NULL );
  tm * now = localtime( &seconds );

  memset( OWNER->thisfilename, 0, 128 * sizeof(char) );

  MMGetStringSymbol("exprunner_filename", basefilename, "experiment%d_%t");

  while( basefilename[i]) { 

    while( basefilename[i] && ( basefilename[i] != '%' ) ) { 
      OWNER->thisfilename[j++] = basefilename[i++]; 
    }
    
    if( !basefilename[i++] ) 
      break;

    memset( temp, 0, 128 * sizeof( char ) ); // reset temp string

    if( basefilename[i] == 'd' )  {

      // use format yyyymmdd
      sprintf( temp, "%04d%02d%02d", 
               ( now->tm_year + 1900 ), ( now->tm_mon + 1 ), now->tm_mday );
      strcat( OWNER->thisfilename, temp );

    } else if( basefilename[i] == 't' ) { // end if 'd'

      // use format hhmm
      sprintf( temp, "%02d%02d", now->tm_hour, now->tm_min );
      strcat( OWNER->thisfilename, temp );

    } else { // end if 't'  

      // should never reach this point
      MMFatalError( "ExpRunner::InitState::entry",
                    "Invalid character in exprunner_filename" ); 

    } // end else-ifs

    i++;
    j = strlen( OWNER->thisfilename ); // reset pointer to end of string

  } // end while

  // Calls the User defined Entry function for Init
  // This is where setParameterNames must be called (if ever)
  OWNER->setParamsCalled = 0;
  OWNER->initEntry( OWNER->expNumber );

  // Start putting data in the universal .m file.
  sprintf( OWNER->thisfilename, "%s.m", OWNER->thisfilename );
  instream.open( OWNER->thisfilename );

  OWNER->loggedNames = OWNER->logger->get_variables();

  if ( instream.fail() ) {

    outstream.open( OWNER->thisfilename );
    if (outstream.fail()) {

      MMFatalError( "ExpRunner::InitState::entry",
                    "Cannot open exprunner_filename for writing" );
    }

    //****     Output all of the base text     ****//
    // output logged named variables
    outstream << "% ";

    for (i = 0; i < OWNER->loggedNames.getCount(); i++) {
      outstream << OWNER->loggedNames.get(i) << ',';
    }
    outstream << endl << "% ";
    
    OWNER->setParamsCalled = 1;

    // Ouput parameter names
    for (i = 0; i < OWNER->parameterNames.getCount(); i++) {
      outstream << OWNER->parameterNames.get(i) << ',';
    }

    outstream << "\nfunction [ exp ] = loadvars( range );\n\nif ~exist( 'range', 'var' )\n  range = [];\nend;\n\nexp.requested = range;\n";

    for (i = 0; i < OWNER->loggedNames.getCount(); i++)
      outstream << "exp." << OWNER->loggedNames.get(i) << " = [];\n";

    for (i = 0; i < OWNER->parameterNames.getCount(); i++)
      outstream << "exp." << OWNER->parameterNames.get(i) << " = [];\n";

    for (i = 0; i < OWNER->loggedNames.getCount(); i++)
      outstream << "exp.varnames{" << i + 1 << "} = '" << OWNER->loggedNames.get(i) << "';\n";

    for (i = 0; i < OWNER->parameterNames.getCount(); i++)
      outstream << "exp.paramnames{" << i + 1 << "} = '" << OWNER->parameterNames.get(i) << "';\n";

    outstream << "\nexp_count = 1;\n\n% Count:\n% 0\n";

    outstream.close();

  } else {

	sprintf( msg, "ExpRunner: Opening previous script file \"%s\" for parsing...\n", 
			 OWNER->thisfilename );
	MMMessage( msg );

    // Check to see if the logged variables and the parameters are the same
    instream.seekg(2, ios::beg);
    
    int numberOfEntries;
    // Check logged names
    for (instream.get(ch), numberOfEntries = 0; ch != '\n'; 
         instream.get(ch), numberOfEntries++) {

      if ( numberOfEntries > OWNER->loggedNames.getCount() ) 
        MMFatalError( "ExpRunner::InitState::Entry", 
                      "There are fewer logged variables then defined!" );

      for ( i = 0, temp[0] = ch; temp[i] != ','; ++i, 
             instream.get(temp[i]) );

      temp[i] = '\0';

      if (strcmp( temp, OWNER->loggedNames.get( numberOfEntries ) ) != 0 )
        MMFatalError( "ExpRunner::InitState::Entry", 
                      "Logged variables different from defined variables!" );
    }

    if ( numberOfEntries != OWNER->loggedNames.getCount() ) 
      MMFatalError( "ExpRunner::InitState::Entry", 
                    "Different # of defined and logged variables!" );
    
    instream.seekg( 2, ios::cur );

    // Check paramater names
    for ( instream.get(ch), numberOfEntries = 0; ch != '\n'; 
          instream.get(ch), numberOfEntries++ ) {

      if ( numberOfEntries > OWNER->parameterNames.getCount() ) 
        MMFatalError( "ExpRunner::InitState::Entry", 
                      "There are fewer Parameter variables than defined!" );

      for ( i = 0, temp[0] = ch; temp[i] != ','; ++i, instream.get(temp[i]) );

      temp[i] = '\0';

      if ( strcmp(temp, OWNER->parameterNames.get(numberOfEntries)) != 0 )
        MMFatalError( "ExpRunner::InitState::Entry", 
                      "Parameter variables different from defined params!" );
    }

    if ( numberOfEntries != OWNER->parameterNames.getCount() ) 
      MMFatalError( "ExpRunner::InitState::Entry", 
                    "Different # of Parameter variables and defined params!" );
    
    // Get the current experiment number
    ch = ' ';
    int j;

    for(j = -2; ch != '%'; j--) {

      instream.seekg( j, ios::end );
      ch = instream.get();
    }

    instream.get( ch );

    instream >> OWNER->expNumber;

    instream.close();
  }

  OWNER->status = INIT; 
}

void ExpRunner::InitState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::InitState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->skipEv->check() ) {

    // Skipping the current experiment
    OWNER->nextExperiment( true );
  }
}


//******************
// IdleState
//******************
void ExpRunner::IdleState::entry ( void ) { 

  EXP_DEBUG_MSG( "ExpRunner::IdleState::entry\n" );

  OWNER->idleEntry( OWNER->expNumber );

  OWNER->status = IDLE; 
}
void ExpRunner::IdleState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::IdleState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->skipEv->check() ) {
    // Skipping the current experiment
    OWNER->nextExperiment( true );
  }
}

//******************
// SetupState
//******************
void ExpRunner::SetupState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::SetupState::entry\n" );

  OWNER->setupEntry( OWNER->expNumber );

  OWNER->status = SETUP; 
} 
void ExpRunner::SetupState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::SetupState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {

    MMMessage( "ExpRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );

  } else if (OWNER->repeatEv->check() ) {

    MMMessage( "ExpRunner: Repeating single experiment on signal\n" );
    OWNER->handleRepeat( OWNER->expNumber );

  }
}

//******************
// ExperimentState
//******************
void ExpRunner::ExperimentState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::ExperimentState::entry\n" );

  OWNER->experimentEntry( OWNER->expNumber );

  // Grab and Reset the logger for new data
  if ( OWNER->logFlag ) {
    MMGrabModule( OWNER->logger, OWNER );
    OWNER->logger->reset();
  }

  OWNER->status = RUNNING; 
}
void ExpRunner::ExperimentState::exit ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::ExperimentState::exit\n" );

  // Done with the experiment. Stop the logger
  if ( OWNER->logFlag )
    MMReleaseModule( OWNER->logger, OWNER );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {

    MMMessage( "ExpRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );

  } else if (OWNER->repeatEv->check() ) {

    MMMessage( "ExpRunner: Repeating single experiment on signal\n" );
    OWNER->handleRepeat( OWNER->expNumber );
  } else

    OWNER->experimentExit( OWNER->expNumber );
}

/*******************
 * WrapupState
 ******************/
void ExpRunner::WrapupState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::WrapupState::entry\n" );

  OWNER->status = WRAPUP; 
}
void ExpRunner::WrapupState::exit ( void ) { 

  EXP_DEBUG_MSG( "ExpRunner::WrapupState::exit\n" );

  // Check which event caused the transition
  if ( OWNER->abortEv->check() ) {

    MMMessage( "ExpRunner: Aborting experiment batch on signal!\n" );
    OWNER->handleAbort( OWNER->expNumber );

  } else if (OWNER->repeatEv->check() ) {

    MMMessage( "ExpRunner: Repeating single experiment on signal\n" );
    OWNER->handleRepeat( OWNER->expNumber );
  } else {

    // Proceed with the next experiment
    OWNER->wrapupExit( OWNER->expNumber );
    OWNER->nextExperiment( false );
  }
}

//******************
// FinishedState
//******************
void ExpRunner::FinishedState::entry ( void ) { 
  EXP_DEBUG_MSG( "ExpRunner::FinishedState::entry\n" );

  OWNER->status = DONE; 
}

// ---------------------------------------------------------------------------
// ExpRunner methods ---------------------------------------------------------
// ---------------------------------------------------------------------------

ExpRunner::ExpRunner( char *name, uint dataLen ) 
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
  initState = new InitState( this );
  idleState = new IdleState( this );
  setupState = new SetupState( this );
  experimentState = new ExperimentState( this );
  wrapupState = new WrapupState( this );
  finishedState = new FinishedState( this );

  // ************************************************************
  // add transitions ( this defines the structure of the machine )
  // 
  //           From State        Event              To State
  //-------------------------------------------------------------

  Transition ( initState,       setupBeginEv,      setupState );
  Transition ( initState,       skipEv,            idleState );
  Transition ( initState,       abortEv,           finishedState );

  Transition ( idleState,       setupBeginEv,      setupState );
  Transition ( idleState,       skipEv,            idleState );
  Transition ( idleState,       abortEv,           finishedState );
  Transition ( setupState,      experimentBeginEv, experimentState );
  Transition ( setupState,      abortEv,           finishedState );
  Transition ( setupState,      repeatEv,          idleState );
  Transition ( experimentState, experimentEndEv,   wrapupState );
  Transition ( experimentState, abortEv,           finishedState );
  Transition ( experimentState, repeatEv,          idleState );
  Transition ( wrapupState,     wrapupEndEv,       idleState );
  Transition ( wrapupState,     abortEv,           finishedState );

  Transition ( idleState,       terminateEv,       finishedState );
  Transition ( initState,       terminateEv,       finishedState );

  // define state machine
  initialize ( initState );

  // Default configuration
  logPeriod = 5;
  logFlag = false;

  // Start with no log messages
  firstLog = lastLog = NULL;

  // Initialize the dynamic data
  expNumber = 0;
  status = IDLE;
}

ExpRunner::~ExpRunner( ) {

  // deallocate events
  if ( setupBeginEv ) delete setupBeginEv;
  if ( experimentBeginEv ) delete experimentBeginEv;
  if ( experimentEndEv ) delete experimentEndEv;
  if ( wrapupEndEv ) delete wrapupEndEv;
  if ( terminateEv ) delete terminateEv;
  if ( abortEv ) delete abortEv;
  if ( skipEv ) delete skipEv;

  // deallocate states
  if ( idleState ) delete idleState;
  if ( setupState ) delete setupState;
  if ( experimentState ) delete experimentState;
  if ( wrapupState ) delete wrapupState;
  if ( finishedState ) delete finishedState;

  // Deallocate the experiment logger
  if ( logger ) delete logger;

}

void ExpRunner::setLogFlag( bool logging )
{
  logFlag = logging;
}

// void ExpRunner::configure( char *n, bool logging, MM_STEP p ) { 

//   baseName = n; 

//   logPeriod = p; 
  
//   logFlag = logging; 

// };

void ExpRunner::logMessage( char *msg ) {
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

void ExpRunner::init ( void ) {

  // Create the logger
  logger = (RHexLogger*) MMFindModule( "rhexlogger", 0 );
  if (!logger) MMFatalError("ExpRunner::init", "Cannot find RHexLogger");

  StateMachine::init();


  logger->reset();

  // Record the starting time of the batch for generating filenames
  startTime = time( NULL );

}

void ExpRunner::activate ( void ) {

  StateMachine::activate();

}

void ExpRunner::deactivate ( void ) {

  StateMachine::deactivate();

}

// ExpRunner::dumpLog : Appends the current log messages to the log file */
void ExpRunner::dumpLog( void ) 
{
  char *timestr;
  logEntry *curEntry;
  time_t curTime;

  // Get the calendar time and print it to the log file.
  curTime = time( NULL );
  timestr = ctime( &curTime );
  // delete the trailing newline on the string
  timestr[ strlen(timestr) - 1 ] = 0;

  outstream << "\n%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
            << "% Experiment #" << expNumber << ". Date: " << timestr << endl;

  // Dump all the log messages to the file, assuming that the first
  // message gives a comma sepaerated parameter list
  if ( firstLog != NULL ) {
    curEntry = firstLog->next;

    while ( curEntry != NULL ) {
      outstream << "% " << curEntry->msg << endl;
      curEntry = curEntry->next;
    }
  }

  // Clear the current log messages
  clearLog();
}

// ExpRunner::clearLog : Clears the current list of log messages */
void ExpRunner::clearLog( ) {
  logEntry *cur = firstLog, *next;

  while ( cur != NULL ) {
    if ( cur->msg ) delete[] cur->msg;
    next = cur->next;
    delete cur;
    cur = next;
  }

  firstLog = lastLog = NULL;

}

// ExpRunner::nextExperiment :
//
// Proceeds with the next experiment after dumping the DataLogger data
// and appending the log messages on the experiment log file
void ExpRunner::nextExperiment( bool skip ) {
  char msg[128];
  char filename[128];
  char basename[128];

  int i;


  if ( logFlag ) {
    // Construct the filename for the data output
    if ( !skip ) {
      logger->exportToFile();
      //      logger->exportViaNet();

      strcpy(basename, logger->get_thisfilename() );

      sprintf( filename, "%s", basename );
      sprintf( msg, 
               "ExpRunner: Saving exp %i data in \"%s.data\"...", 
               expNumber, filename );
      MMMessage( msg );

      // Log the experiment details to the log file to a param file in
      // Matlab loadable format

      outstream.open( thisfilename, ofstream::out | ofstream::app );
      if (outstream.fail()) {

        MMFatalError( "ExpRunner::nextExperiment", 
                      "Cannot open exprunner_filename for writing!" );
      }

      dumpLog();
      
      outstream << "if (isempty( range ) | ~isempty ( find ( range == "
                << expNumber + 1 << " ) ))\n"
                << "\n  if exist ( '" << filename <<  ".m', 'file')\n"
                <<"    disp ( 'Reading data file: " << filename 
                <<  ".m ...' );\n\n"
                << "    " << filename << ";\n";

      for ( i = 0; i < loggedNames.getCount(); i++ ) {

        outstream << "    exp." << loggedNames.get(i) << "{exp_count} = " 
                  << loggedNames.get(i) << ";\n";
      }

      if ( parameterNames.getCount() != parameterValues.getCount() )
        MMFatalError( "ExpRunner::nextExperiment", 
                      "Parameter values must be set before values can be logged");
      for ( i = 0; i < parameterNames.getCount(); i++ ) {
        outstream << "    exp." << parameterNames.get(i) << "(exp_count) = " 
                  << parameterValues.get(i) << ";\n";
      }

      outstream << "    exp.expnumbers(exp_count) = " << expNumber << ";\n"
                << "    exp_count = exp_count+1;\n"
                << "  else\n"
                << "    error ( 'Data file " << filename 
                << ".m does not exist' );\n"
                << "  end;\n"
                << "end;\n\n"
                << "% Count:\n"
                << "% " << expNumber+1 << endl;

      outstream.close();

    } else {
      sprintf( msg, "ExpRunner: Skipping experiment %i, Data not saved!...", expNumber );
      MMMessage( msg );
      logMessage( "Experiment skipped!\n" );
    }

  } else {

    if ( !skip )
      sprintf( msg, "ExpRunner: Experiment %i completed. Cleaning up...", expNumber );

    else 
      sprintf( msg, "ExpRunner: Experiment %i skipped. Cleaning up...", expNumber );

    MMMessage( msg );
  }

  // Compute new experiment number
  expNumber++;

  MMMessage( "done.\n" );
}

void ExpRunner::setParameters( Strings names ) {

  if ( setParamsCalled ) {

    MMFatalError( "ExpRunner::setParameters", 
                  "setParameters can only be called once and in Init::Entry!");
    return;
  }

  parameterNames = names;
}

void ExpRunner::setParameterValues( Floats values ) {

  if ( values.getCount() != parameterNames.getCount() )

    MMFatalError( "ExpRunner::setParameterValues", 
                  "length of parameter values and names different." );

  parameterValues = values;
}
