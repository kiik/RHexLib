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
 * $Id: ModuleManager.hh,v 1.5 2001/08/15 15:58:52 ulucs Exp $
 *
 * Module Manager Header File
 *
 * This file contains all datatypes and function prototypes for 
 * the Module Manager. The functions declared in this file are
 * implimented in ModuleManager.cc and Module.cc
 *
 * Created       : Eric Klavins, 10/16/2000
 * Last Modified : Eric Klavins, 07/19/2001
 *
 ********************************************************************/

#ifndef _MODULEMANAGER_HH
#define _MODULEMANAGER_HH

#include "types.hh"
#include "Hardware.hh"
#include "SymbolTable.hh"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <malloc.h>

#include <unistd.h>
#include <time.h>

#ifdef _QNX4_
#include <process.h>
#include <sys/proxy.h>
#include <sys/kernel.h>
#include <sys/sched.h>
#include <signal.h>
#include <semaphore.h>
#endif

// Module specific constants ---------------------------
#define  SINGLE_USER     true   // Module ownership states
#define  MULTI_USER      false
#define  PERIODIC_UPDATE false  // Module update mode
#define  POLLING_UPDATE  true

#ifdef _QNX4_

// Threading related constants
#if defined(__386__)
    #define FAR
    #define POLLING_STACK_SIZE 8192
#else
    #define FAR      __far
    #define POLLING_STACK_SIZE 4096
#endif

#endif

// Module specific data types ------------------------- 
typedef int MM_STATUS;
typedef unsigned long int MM_STEP; 

// The Module class ------------------------------------
class Module {
  
public:
  
  // Constructor and destructors
  Module( char *name, int index, bool singleUser, bool polling );
  virtual ~Module() {};

  // Field access functions
  char   *getName( void ) { return name; };
  int     getIndex( void ) { return index; };

  typedef enum { UNINIT, INACTIVE, ACTIVE } ModuleState;

  ModuleState getState ( void ) { return state; };
  MM_STEP getPeriod( void ) { return period; };
  MM_STEP getOffset( void ) { return offset; };
  int     getOrder( void ) { return order; };

  Module *getOwner( void ) { return usedby; };
  
  bool    isPolling( void ) { return polling; };
  bool    isSingleUser( void ) { return single_user; };
  
  // methods which must be defined by the module programmer. 
  // These are pure virtual functions
  virtual void init( void ) = 0;
  virtual void uninit( void ) = 0;
  virtual void activate( void ) = 0;
  virtual void deactivate( void ) = 0;
  virtual void update( void ) = 0;
  
 private:
  
  friend class ModuleManager;
  
  // The name of a module: module makers must allocate this string
  char * name;

  // The index or minor number of the module (e.g. for motors 1 through 6)
  int index;
 
  // state of the module
  ModuleState state;
  
  // whether or not this module should only be used by one other module
  bool single_user;
  
  // whether or not this module is a polling module
  bool polling;
  
  // for module_actuator and module_controller only
  Module * usedby;

  // Count to keep track of how many times this module has been
  // grabbed and released.
  int grabCount;

  // the order of the module in the module (or polling) list:
  // determines execution order
  int order;
  
  // determine update frequency and offset. and MM_STEP is one step
  // through the module manager's main loop wherein each ready
  // module is updated
  MM_STEP period, offset;   
  
  // used to keep a linked list of modules
  Module * next, * prev; 
  
  // Some linked list utility functions
  void addToList ( Module ** head );
  void removeFromList ( Module ** head );
  Module *find_module_in_list( char *name, int index );

  Module *getNext( void ) { return next; };
  Module *getPrev( void ) { return prev; };
  
  void setState ( ModuleState s ) { state = s; };
  void setPeriod ( MM_STEP p ) { period = p; };
  void setOffset ( MM_STEP o ) { offset = o; };
  void setOrder ( int o ) { order = o; };
  void setOwner ( Module *m ) { usedby = m; };

protected:
  void setName ( char * str ) { name = str; };
  
};  

// The Module Manager class -------------------------------------
class ModuleManager {

public:
  
  // Constructors and Destryuctor
  ModuleManager ( void );
  ~ModuleManager ();
  
  // Data access functions
  MM_STEP getStepCount ( void ) { return current_step; };
  CLOCK getStepPeriod ( void ) { return step_period; };

  // Module manager controls
  void mainLoop( void );
  void shutdown( void );
  void poweroff( void );
  
  // Module list management
  void addModule( Module *, MM_STEP, MM_STEP, int );
  void removeModule( Module * );
  Module * findModule( char *, int );

  // Low level hardware interface
  void chooseHardware( Hardware *hw );
  Hardware *getHardware( void );
  // Warning: The readClock() function does not check whether hardware
  // is selected
  CLOCK readClock( void ) { return hardware->readClock(); };
  CLOCK readUClock( void ) { return hardware->readUClock(); };
  
  // Configuration management
  void readConfigFile( const char *filename );
  void printSymbols( void ) { st.printSymbols(); };
  float getFloatSymbol( const char *n, const float def ) {
    return st.readFloat( n, def );
  };
  Floats getArraySymbol( const char *n ) { return st.readArray( n ); };
  void getStringSymbol( const char *n, char *str, const char *def ) { 
    st.readString( n, str, def ); 
  };
  Strings getStrArraySymbol( const char *n ) { return st.readStrArray( n ); };

  // Module state handling
  void activateModule( Module * );
  void deactivateModule( Module * );
  void grabModule( Module *, Module * );
  void releaseModule( Module *, Module * );
  
  // Error messages and such.
  void fatalError( char *, char *);
  void warning( char *, char * );
  void message( char * );
  
  // Debugging
  void printModules( void );
  
private:

  // Pointer to the beginning of the normal module list
  Module * head;
  
  // Pointer to the beginning of the polling module list
  Module * polling_head;
  
  // The current module manager number
  MM_STEP current_step;
  
  // timing data
  CLOCK clock_mark;
  CLOCK step_period;
  
  // Low level Hardware interface to be used by the modules
  Hardware *hardware;

  // Symbol table to hold configuration data
  SymbolTable st;

  // Threading related variables
#ifdef _QNX4_

  int    thread_enable;
  int    main_loop_prio;
  int    polling_high_prio;
  int    polling_low_prio;

  sem_t  sem_poll;  // Semaphore and flag to kill threaded process.
  bool   child_killflag; 

#endif

  void updatePollingModules( void );
  void updateModules( void );

  // child function which run polling modules in threaded implementation

#ifdef _QNX4_

  static FAR void pollingThread( void FAR *parm );
  void mainLoopThreaded ( void ); // threaded main loop

#endif
  
  void mainLoopSingleThread ( void ); // normal main loop

};  

// The main ModuleManager class instantiation extern declaration
extern ModuleManager    mm;

// Wrapper functions for the main module manager so that users don't
// have to know about the global variable "mm"
inline void MMMainLoop( void ) { mm.mainLoop(); };
inline void MMShutdown( void ) { mm.shutdown(); };
inline void MMPowerOff( void ) { mm.poweroff(); };
inline void MMAddModule( Module * m, MM_STEP p, MM_STEP off, int ord ) { 
  mm.addModule( m, p, off, ord ); 
};
inline void MMRemoveModule( Module * m ) { mm.removeModule ( m );};
inline Module * MMFindModule( char *n, int ind ) { 
  return mm.findModule ( n, ind );
};
inline void MMChooseHardware( Hardware *hw ) { mm.chooseHardware( hw );};
inline Hardware *MMGetHardware( void ) { return mm.getHardware();};

inline void MMActivateModule( Module * m ) { mm.activateModule ( m );};
inline void MMDeactivateModule( Module * m ) { mm.deactivateModule ( m );};
inline void MMGrabModule( Module * m, Module * ask ) { 
  mm.grabModule( m, ask );
};
inline void MMReleaseModule( Module * m, Module * ask ) {
  mm.releaseModule( m, ask );}
;
inline void MMFatalError( char *fn, char *msg ) { mm.fatalError ( fn, msg ); };
inline void MMWarning( char *fn, char *msg ) { mm.warning ( fn, msg ); };
inline void MMMessage( char *msg ) { mm.message ( msg ); };
inline void MMPrintModules( void ) { mm.printModules(); };
inline CLOCK MMGetStepPeriod( void ) { return mm.getStepPeriod(); };
inline MM_STEP MMGetStepCount( void ) { return mm.getStepCount(); };
inline CLOCK MMReadClock( void ) { return mm.readClock(); };
inline CLOCK MMReadUClock( void ) { return mm.readUClock(); };
inline double MMReadTime( void ) { return CLOCK_TO_SEC( mm.readClock() ); };
inline double MMReadUTime( void ) { return CLOCK_TO_SEC( mm.readUClock() ); };

inline void MMReadConfigFile( const char *fn ) { mm.readConfigFile( fn ); };
inline void MMPrintSymbols( void ) { mm.printSymbols(); };
inline float MMGetFloatSymbol( const char *n, const float def ) { 
  return mm.getFloatSymbol( n, def );
};
inline Floats MMGetArraySymbol( const char *n ) { 
  return mm.getArraySymbol( n ); 
};
inline void MMGetStringSymbol( const char *n, char *str, const char *def ) {
  mm.getStringSymbol( n, str, def ); 
};
inline Strings MMGetStrArraySymbol( const char *n ) { 
  return mm.getStrArraySymbol( n ); 
};
#endif


