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
 * $Id: ModuleManager.cc,v 1.7 2001/08/15 16:41:26 ulucs Exp $
 *
 * ModuleManager class function definitions
 *
 * This file contains the implementation of the functions defined
 * in ModuleManager.hh
 *
 * Created       : Eric Klavins, 10/16/2000
 * Last Modified : Eric Klavins, 07/19/2001
 *
 ********************************************************************/

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _QNX4_
#include <sys/sched.h>
#include <unistd.h>
#endif

// Local includes
#include "ModuleManager.hh"
#include "Hardware.hh"

// The main module manager instantiation -- a global variable, but
// users don't need to know about it.
ModuleManager   mm;

// ModuleManager::ModuleManager: Class constructor. Initializes the
// Module Manager
ModuleManager::ModuleManager ( void ) {

  head = polling_head = NULL;
  current_step = 0;
  
  // The default step period is 1ms = 1000us.
  step_period = 1000;
  clock_mark = 0;

  hardware = NULL;
}

// ModuleManager::~ModuleManager: Class destructor. Destroys the class object
ModuleManager::~ModuleManager ( void ) {

  // Cleanly shutdown all existing modules
  shutdown ();

}

// ModuleManager::chooseHardware: 
//
// Selects the supplied low level hardware interface. This function
// can be called only once, during initialization
void ModuleManager::chooseHardware( Hardware *hw ) {

  if ( hardware != NULL )
    fatalError( "ModuleManager::chooseHardware", 
                "Low level hardware interface already selected!" );

  hardware = hw;

  // Perform proper initialization
  hardware->initialize();
}

// ModuleManager::getHardware: 
//
// Returns the selected low level hardware interface object. Exits
// with an error if non is chosen.
Hardware *ModuleManager::getHardware( void ) {

  if ( hardware == NULL )
    fatalError( "ModuleManager::getHardware", 
                "No low level hardware interface selected!" );

  return hardware;
}

// ModuleManager::addModule: Adds a module to the module manager's
// list by order
void ModuleManager::addModule ( Module *m, MM_STEP period, MM_STEP offset, int order ) {

  char msg[1128];

  // Check if the given module parameters are valid. 
  // If not, exit with an error message.
  if ( period < 1 ) {
    sprintf( msg, "Module period must be greater than 0 !" );
    fatalError ( "ModuleManager::addModule", msg );
  };
  if ( offset >= period ) {
    sprintf( msg, "Module offset must be smaller than the period !" );
    fatalError ( "ModuleManager::addModule", msg );
  };

  // First, look if the module already exists
  if ( findModule ( m->getName(), m->getIndex() ) != NULL) {
    sprintf( msg, "Module %s[%i] exists!", m->getName(), m->getIndex() );
    fatalError ( "ModuleManager::addModule", msg );
  }

  // Fill in the timing and order fields
  m->setPeriod ( period );
  m->setOffset ( offset );
  m->setOrder ( order );

  // Add to the Module List
  if ( m->isPolling() == false )
    m->addToList ( &head );  
  else
    m->addToList ( &polling_head );

  // Execute the init function
  m->init();
  m->setState ( Module::INACTIVE );

}

// ModuleManager::readConfigFile : Reads in a configuration file into
// the symbol table
void ModuleManager::readConfigFile( const char *filename ) { 

  char msg[128];

  sprintf( msg, "Loading configuration file %s...", filename );
  message( msg );

  st.loadFile( filename ); 

  message( "done.\n" );

};

// ModuleManager::printModules: For debugging purposes
void ModuleManager::printModules ( void ) {

  Module *temp = head;
  
  printf ( "Normal modules:\n" );
  if ( temp != NULL ) {

    do {

      printf( "  %s:\t %d %s[%d], %ld, OWNER:%s(grab = %i)\n", 
              ( temp->getState() == Module::ACTIVE ) ? "ACTIVE" : "INACTIVE",
              temp->getOrder(), temp->getName(), 
              temp->getIndex(), temp->getPeriod(),
              ( temp->isSingleUser() ) 
                ? ( ( temp->getOwner() ) ? temp->getOwner()->getName() : 
                    "none") 
                : "MULTI", temp->grabCount 
              );

      temp = temp->getNext();

    } while ( temp != head );

  }

  temp = polling_head;
  printf ( "Polling modules:\n" );

  if ( temp != NULL ) {

    do {

      printf ( "  %s:\t %s[%i], %d\n", 
               ( temp->getState() == Module::ACTIVE ) ? "ACTIVE" : "INACTIVE",
               temp->getName(), temp->getIndex(), temp->getOrder() );
      temp = temp->getNext();

    } while ( temp != polling_head );

  }

}

// ModuleManager::removeModule: 
//
// Removes a module from the module list.
void ModuleManager::removeModule ( Module *m ) {

  if ( m->getOwner() != NULL )
    fatalError( "ModuleManager::removeModule", "Module in use!" );

  // Deactivate module if necessary.
  deactivateModule ( m );

  // Remove from the appropriate List
  if ( m->polling == false )
    m->removeFromList ( &head );  
  else
    m->removeFromList ( &polling_head );

  // Execute the uninit function
  m->uninit();
  m->setState ( Module::UNINIT );
  
}

// ModuleManager::activateModule: 
//
// Puts a module in the ACTIVE state and calls activate()
void ModuleManager::activateModule ( Module *m ) {

  if ( m->getState() == Module::ACTIVE )
    return;
  else if ( m->getState() == Module::UNINIT )
    fatalError( "ModuleManager::activateModule", "Module uninitialized!" );

  m->setState ( Module::ACTIVE );
  m->activate();

}

// ModuleManager::deactivateModule: 
//
// Puts a module in the INACTIVE state and calls deactivate()
void ModuleManager::deactivateModule ( Module *m ) {

  if ( m->getOwner() != NULL )
    fatalError( "ModuleManager::deactivateModule", "Module in use!" );

  else if ( m->getState() == Module::UNINIT || m->getState() == Module::INACTIVE ) 
    return;

  else {
    m->setState ( Module::INACTIVE );

    // Call deactivate()
    m->deactivate();

  }

}

// ModuleManager::grabModule: Locks a single user module.
//
// note: Also calls activateModule () for the module.
void ModuleManager::grabModule ( Module *m, Module *asker ) {

  if ( m->getState() == Module::UNINIT )
    fatalError( "ModuleManager::grabModule", "Module uninitialized!" );

  else if ( m->getOwner() != NULL ) {

    char str[200];

    sprintf ( str, "Module %s(%d) in use! (asker is %s(%d) but owner is %s(%d))", 
              m->getName(), m->getIndex(), asker->getName(), 
              asker->getIndex(), m->usedby->getName(), m->usedby->getIndex() );
    fatalError( "ModuleManager::grabModule", str );

  }

  else {

    // Activate the module if not already active
    activateModule ( m );

    // If there is a single user requirement, set the usedby field
    if ( m->isSingleUser() == true )
      m->setOwner ( asker );

    m->grabCount++;
  }
}

// ModuleManager::releaseModule: Releases the lock on a module.
//
// note: The module is deactivated when all modules which previously
// grabbed it release it
void ModuleManager::releaseModule ( Module *m, Module *asker ) {

  // Update the grab count for this module.
  if ( m->grabCount != 0 )
    m->grabCount--;

  if ( m->getOwner() == asker )
    m->setOwner ( NULL );

  else if ( m->getOwner() != NULL )
    warning( "ModuleManager::releaseModule", "Not the owner!" );

  // Deactivate the module if everybody released it
  if ( m->grabCount == 0 )

    deactivateModule( m );
}

// Module::find_module_in_list: 
//
// Finds a module by name and index in a given list
Module *Module::find_module_in_list( char *name, int index ) {

  Module *current = this;

  if ( current != NULL ) {

    do {

      // If the name and index both match, module is found
      if ( !strcmp ( name, current->getName() ) && ( current->getIndex() == index) )
        return current;

      current = current->getNext();

    } while ( current != this );

  }

  // No success.
  return NULL;
}

// ModuleManager::findModule: Finds a module by name and index
Module *ModuleManager::findModule ( char *name, int index ) {

  Module *m;

  // Look for the module in the list of normal modules
  if ( (m = head->find_module_in_list ( name, index )) != NULL )
    return m;

  // Look for the module in the list of polling modules
  return polling_head->find_module_in_list ( name, index );

}

// Modulemanager::updatePollingModules: 
//
// Goes through the list of polling modules and calls update for each
// of them.
void ModuleManager::updatePollingModules( void ) {

  Module *current;

  current = polling_head;

  if ( current != NULL ) {

    do {

      if ( current->getState() == Module::ACTIVE)
        current->update();

      current = current->getNext();

    } while ( current != polling_head );

  }

}

#ifdef _QNX4_
static FAR void ModuleManager::pollingThread( void  FAR *parm ) {

  char * FAR *argv = (char * FAR *) parm;

  while( !mm.child_killflag ) {

    setprio( *_threadid, mm.polling_high_prio ); 

    // Grab the semaphore to prevent the main loop to mess with the modules
    sem_wait( &mm.sem_poll );

    // Perform the update
    mm.updatePollingModules();

    // Release the semaphore.
    sem_post( &mm.sem_poll );

    // Set the priority back to a lower value
    setprio( *_threadid, mm.polling_low_prio );
  }

  _endthread();
}
#endif

// ModuleManager::updateModules: 
//
// Goes through the list of normal modules and calls update for each
// of them. Also calls the update function for the polling modules as
// frequently as possible.
void ModuleManager::updateModules( void ) {

  Module *current;

  current = head;

  if ( current != NULL ) {

    do {

      if ( ( current_step - current->getOffset() ) 
           % current->getPeriod() == 0 ) {

        if ( current->getState() == Module::ACTIVE) {
          //printf( "Updating module %s[%i]\n", current->getName(), 
          //                                    current->getIndex());
          current->update();
        }
    }

    current = current->getNext();

    // Update all the nmodules in the polling list
    // ulucs: I had to take this out because it took WAAAAY too much time to
    // do it this way
    //    updatePollingModules ();

    } while ( current != head );

  }

}

//ModuleManager::mainLoop: The main loop of the manager.
void ModuleManager::mainLoopSingleThread ( void ) {

  int   count = 0;
  CLOCK curTime;
  CLOCK startTime;

#ifdef _QNX4_
  // Set the priority of the main loop thread
  //  setprio( getpid(), main_loop_prio );
#endif

  startTime = hardware->readClock();

  //  while ( count < 50000 ) {
  while ( 1 ) {

    curTime = hardware->readClock();

    if ( curTime - clock_mark >= step_period ) {
      // printf ( "new step: %ld, %ld, %ld\n", current_step, count, curTime );
 
      while ( curTime - clock_mark >= step_period ) {
        clock_mark += step_period;
        current_step++;
      } 

      updateModules ();
 
      count++;
    } 
 
  
    // Update all the modules in the polling list
    updatePollingModules ();
  } 

  // curTime = hardware->readClock();

  //printf ( "Total number of steps: %ld, difference: %f\n", current_step, 
  //           CLOCK_TO_DOUBLE( curTime - startTime ) / count );

}


void ModuleManager::mainLoop ( void ) {

  // Check whether a low level hardware interface is chosen
  getHardware();

#ifdef _QNX4_

  // Note: thread enable set to one and when run on qnx will enable
  // the threaded version of the module manager, also the priorities of the
  // polling threads must be lower than that of the main thread for the
  // program not to lock

  thread_enable = MMGetFloatSymbol( "mm_thread_enable", 0.0 );
  if( thread_enable != 0 && thread_enable != 1) 
    MMFatalError( "ModuleManager::mainLoop", 
                  "Invalid value for thread_enable!" );
 
  // The main loop priority will be used for both single and
  // multithreaded operations.

  main_loop_prio  = MMGetFloatSymbol( "mm_mainloop_prio", 25.0 );
  if ( main_loop_prio <= 10.0 ) 
    MMFatalError( "ModuleManager::mainLoop", 
                  "Invalid value for mm_mainloop_prio!" );

  if ( thread_enable ) {
    // If threads are enabled, retrieve thread configuration parameters

    polling_high_prio  = MMGetFloatSymbol( "mm_polling_highprio", 24 );
    if( ( polling_high_prio < 10.0) 
        || (polling_high_prio >= main_loop_prio ) ) 
      MMFatalError( "ModuleManager::mainLoop", 
                    "Invalid value for mm_polling_highprio!" );

    polling_low_prio  = MMGetFloatSymbol( "mm_polling_lowprio", 19 );
    if( polling_low_prio > polling_high_prio )
      MMFatalError( "ModuleManager::mainLoop", 
                    "Invalid value for mm_polling_lowprio" );

  } else {
    // Otherwise, set some dummy values

    polling_low_prio = 19.0;
    polling_high_prio = 19.0;
  }

  if( thread_enable == 1 )

    mainLoopThreaded();

  else 

    mainLoopSingleThread();

#else

  mainLoopSingleThread();

#endif
}

#ifdef _QNX4_

void ModuleManager::mainLoopThreaded ( void ) {

  pid_t    proxy;
  timer_t  id;
  struct   itimerspec timer;
  struct   sigevent event;
  timespec res;

  res.tv_sec = 0;
  res.tv_nsec = 1000000;

  sem_init(&sem_poll, 1, 1);

  // Get a proxy for the timer to kick
  proxy = qnx_proxy_attach( 0, 0, 0, -1 );
  if( proxy == -1 )
    MMFatalError( "ModuleManager::mainLoopThreaded", 
                  "Unable to attach proxy!" );

  //clock_setres(CLOCK_REALTIME, &res);

  // Attach the proxy to the timer
  event.sigev_signo = -proxy;
  id = timer_create( CLOCK_REALTIME, &event );
  if( id == -1 )
    MMFatalError( "ModuleManager::mainLoopThreaded", 
                  "Unable to attach timer." );
  
  
  // Setup the timer intervals:
  //
  // 10 milliseconds before initial firing ,
  // 1 millisecond repetitive timer afterwards.
  timer.it_value.tv_sec     = 0L;
  timer.it_value.tv_nsec    = 10000000L;
  timer.it_interval.tv_sec  = 0L;
  timer.it_interval.tv_nsec = 1000000;
  timer_settime( id, 0, &timer, NULL );

  //
  // The following section of code creates the polling thread
  //
  char *args[3];
  char *stack;
  int   tid;
  
  args[0] = "pollingThread";
  args[1] = "parm";
  args[2] = NULL;
  
  // Create a new stack for the polling thread
#if defined(__386__)
  stack = (char *) malloc ( POLLING_STACK_SIZE );
#else 
  stack = (char *) _nmalloc( POLLING_STACK_SIZE );
#endif

  // Set the priority of the main loop thread
  setprio( getpid(), main_loop_prio );
  updateModules (); // ulucs: I don't understand why this is here.

  // Initialize the killflag to false to avoid the polling from exiting
  child_killflag = false;
  tid = _beginthread( pollingThread, stack , POLLING_STACK_SIZE, args );
  
  while ( true ) {
      
    // Grab the semaphore to avoid the polling thread from messing
    // with the modules.
    sem_wait( &sem_poll );

    // Update all the non-polling modules
    updateModules ();

    // Set the priority of the polling thread to its high value
    setprio( tid, polling_high_prio ); 

    // Release the semaphore
    sem_post(&sem_poll);

    // Go to sleep until the next millisecond.
    Receive( proxy, 0, 0 );   
  }
}
#endif

// ModuleManager::shutdown: gently shuts down the module manager by
// deactivating and removing all modules
void ModuleManager::shutdown ( void ) {
    
  Module *current;
    
  while ( ( current = head ) != NULL ) {
      
    // Release the module if it is owned by somebody
    releaseModule ( current, current->getOwner() );

    // Deactivate and remove the module.
    deactivateModule ( current );
    removeModule ( current );
  } 
    
  while ( (current = polling_head) != NULL ) {

    // Release the module if it is owned by somebody
    releaseModule ( current, current->getOwner() );

    // Deactivate and remove the module.
    deactivateModule ( current );
    removeModule ( current );

  } 

  // Cleanup and clear current hardware selection
  if ( hardware != NULL ) {
    hardware->cleanup();
    hardware = NULL;
  }

#ifdef _QNX4_
  // clean up child thread 
  child_killflag = true;
#endif
}

// ModuleManager::poweroff: 
//
// Shuts down the module manager and exits the program
void ModuleManager::poweroff ( void ) {
  shutdown( );
  exit( 0 );
}

// ModuleManager::fatalError: Displays an error message and shuts down
void ModuleManager::fatalError ( char *fn, char *msg) {

  fprintf ( stderr, "Fatal Error in %s: %s\n", fn, msg );
  poweroff();
}

// ModuleManager::warning: Displays a warning message and returns
void ModuleManager::warning( char *fn, char *msg ) {

  fprintf( stderr, "Warning in %s: %s\n", fn, msg );

}

// ModuleManager::message: Displays a message
void ModuleManager::message( char *msg ) {

  printf(msg);

}

