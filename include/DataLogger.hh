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
 * $Id: DataLogger.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The Data Logger base Module. 
 *
 * This file contains the Module class definition, all datatypes
 * and constants associated with the data logger base module type.
 * The data logger is the base class for all data logging facilities. 
 * It is used to periodically record data from the running system and
 * dump it to a file.
 *
 * Created       : Uluc Saranli, 10/29/2000
 * Last Modified : Uluc Saranli, 5/24/2001
 *
 ********************************************************************/

#ifndef _DATALOGGER_HH
#define _DATALOGGER_HH

// -----------------------------------------------------------------
// MODULE : DataLogger
//
// DESCRIPTION:
//
//   The DataLogger class is an abstract base class, defining a 
// standard interface for all data logging facilities in RHexLib.
// A data logging module must be declared as:
//
// class MyDataLogger : public DataLogger {
//
//  public:
//     MyDataLogger( void ) 
//        : DataLogger( numvars, "mylogger", index )
//     { init_stuff(); };
//
//     void fillRecord( float *f ) { f[0] = stuff; };
// }
//
// where the contents of the fillRecord() method will define the 
// things that are to be logged. Note that the number of variables
// to be logged is specified in the constructor of the defined class
// but can be changed with the reset( new_reclen) method of the 
// DataLogger class
//
// NOTES:
//
// - exportToFile( filename, DL_ASCII ) method can be used to output 
// the current contents of the logged variables into a file in ASCII 
// format
//
// - exportToFile( filename, DL_ASCII, var_list ) method can be used 
// to output the current contents of the logged variables into a file 
// in ASCII format, together with a Matlab script to load the data 
// file with the supplied variable names.
//
// - exportViaNet(...) method can be used to output the contents of 
// the logger over FTP. The arguments are the same as exportToFile(...)
//
// - The period of the module, determined with MMAddModule() determines
// how frequently the variables will be logged
//
// - When deactivated, this module will not log any new entries.
//
// -----------------------------------------------------------------

#include "ModuleManager.hh"

// Module specific constants ---------------------------------------

// Number of records in a data chunk.
// Data chunks are groups of records allocated at one time
#define DL_CHUNK_SIZE   ( 1024 )

// Data types ------------------------------------------------------

enum DL_OUTPUT_TYPE { DL_ASCII, DL_BINARY, DL_MATLAB };

class DLDataChunk {

public:
  DLDataChunk ( int reclen );
  ~DLDataChunk ( );

  DLDataChunk   *nextChunk ( void ) { return next; };
  void          append ( DLDataChunk *n ) { next = n; };

  int           dataCount ( void ) { return currec; };
  float         *getRecord ( uint32 num );
  float         *nextRecord ( void ) {
    return (currec < DL_CHUNK_SIZE) ? getRecord ( currec++ ) : NULL; 
  }
 
  void          dumpToFile( FILE *outfile );

  void          restart ( void ) { currec = 0; }

private:

  // next chunk in the list
  DLDataChunk   *next;

  // Size of a record
  int           recordLength;

  // Current record to be filled
  int           currec;

  // Record array
  float         *data;

};

// The DataLogger class -----------------------------------------
class DataLogger : public Module {

public:
  DataLogger ( void );                   // Assumes record length of 1.
  DataLogger ( int reclen );             // Assumes default name and index = 0
  DataLogger ( int reclen, int index );  // Assumes default name
  DataLogger ( int reclen, char *name ); // Assumes index = 0
  DataLogger ( int reclen, char *name, int index );
  virtual ~DataLogger ();
  
  void  init ( void );
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void );

  // Pure virtual function to fill in a record with the desired data.
  virtual void  fillRecord ( float *f ) = 0;

  int  removeFile(char *filename);
  int  exportFileViaNet(char *filename);
  void exportToFile ( char *basefilename, DL_OUTPUT_TYPE type = DL_ASCII,
                      char** names = NULL );
  int  exportViaNet ( char *basefilename, DL_OUTPUT_TYPE type = DL_ASCII, 
                      char **names = NULL );
  void reset( void );
  void reset( int new_reclen );

  void setLoggingPeriod( int p );

private:

  // data members
  int           recordLength;   // Number of doubles in a record
  DLDataChunk  *firstChunk;     // First chunk in the list
  DLDataChunk  *curChunk;       // Current chunk in use
  int           log_period;     // desired logging period
  int           update_count;   // Count of updates since last log.
  int           update_skip;

  // utility functions
  void initialize( int reclen );
  void erase( void );

};

#endif
