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
 * $Id: DataLogger.cc,v 1.3 2001/07/20 04:25:39 ulucs Exp $
 *
 * The data logger Module. Periodically records an array of single 
 * precision numbers. Aloso supports exporting the data to ouput files.
 *
 * This file implements the member functions defined in DataLogger.hh
 *
 * Created       : Uluc Saranli, 10/29/2000
 * Last Modified : Joel Weingarten, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "StdModules.hh"
#include "DataLogger.hh"


// DLDataChunk::DLDataChunk:  Class constructor ------------------------------
DLDataChunk::DLDataChunk( int reclen ) {

  next = NULL;

  recordLength = reclen;

  // Start from the first record
  currec = 0;

  // Allocate the data array
  if ( ( data = new float[ reclen * DL_CHUNK_SIZE] ) == NULL )
    MMFatalError( "DLDataChunk::DLDataChunk" , "Cannot allocate memory!" );

}

// DLDataChunk::~DLDataChunk:  Class destructor
DLDataChunk::~DLDataChunk() {

  next = NULL;

  currec = 0;

  // DeAllocate the data array
  if ( data != NULL )
    delete[] data;

}

// DLDataChunk::getRecord: Returns a pointer to the beginning of a record ---
float *DLDataChunk::getRecord ( uint32 num ) { 

  return ( ( num < DL_CHUNK_SIZE) ? &data[ num * recordLength ] : NULL );

};

void DLDataChunk::dumpToFile( FILE *outfile ) {

  int i, j, recLen = recordLength, dataCnt = currec;

  // Write out all the entries in a record, followed by a newline
  for ( i = 0; i < dataCnt; i++ ) {

    for ( j = 0; j < recLen; j++ ) {

      fprintf( outfile, "%g ", data[ i*recLen + j ] );
    }

    // Next line
    fprintf(outfile, "\n");
  }
}

void DataLogger::initialize( int reclen ) {

  recordLength = reclen;

  // Default to fastest logging
  log_period = 1;
  update_count = 0;
  update_skip = 0;

  // Allocate a new data chunk
  if ( ( curChunk = firstChunk = new DLDataChunk ( reclen ) ) == NULL )
    MMFatalError( "DataLogger::initialize" , "Cannot allocate memory!" );

}

// DataLogger::DataLogger : Class constructors with various arguments--------
DataLogger::DataLogger( void )
  : Module ( DATALOGGER_NAME, 0, true, false ) {

  initialize( 1 );

}
DataLogger::DataLogger( int reclen )
  : Module ( DATALOGGER_NAME, 0, true, false ) {

  initialize( reclen );

}
DataLogger::DataLogger( int reclen, int index )
  : Module ( DATALOGGER_NAME, index, true, false ) {

  initialize( reclen );

}
DataLogger::DataLogger( int reclen, char *name )
  : Module ( name, 0, true, false ) {

  initialize( reclen );

}
DataLogger::DataLogger( int reclen, char *name, int index )
  : Module ( name, index, true, false ) {

  initialize( reclen );

}

// DataLogger::erase : Erases the contents of the data logger by
// deleting all the data chunks and resetting the record count to
// 0. 
void DataLogger::erase ( void ) {

  DLDataChunk   *cur, *next;

  // Delete all the data chunks
  cur = firstChunk;

  while ( cur != NULL ) {
    next = cur->nextChunk();;
    delete cur;
    cur = next;
  }

  firstChunk = curChunk = NULL;
}

// DataLogger::DataLogger : Class destructor --------------------------------
DataLogger::~DataLogger () {

  erase();

}

void DataLogger::setLoggingPeriod( int p ) { 

  if ( p < 1 ) {
    MMWarning( "DataLogger::init", "Attempt to set logging period <= 0!" );
    p = 1;
  }

  log_period = p;

  update_skip = log_period / getPeriod();
};

// DataLogger::init : Gets the desired period from the config file
void DataLogger::init( void ) {

  setLoggingPeriod( int( MMGetFloatSymbol( "logging_period", 1.0 ) ) );

}


// DataLogger::update : Allocates a new record and calls the user defined ----
// fillrecord function to collect the data.
void DataLogger::update( void ) {

  float         *fill;
  DLDataChunk   *newChunk;

  // Skip the necessary number of updates to obtain the required periodicity
  update_count++;
  if ( log_period != 0 && ( update_count < update_skip ) ) return;
  update_count = 0;
       
  // Retrieve the pointer to the next record if possible
  if ( ( fill = curChunk->nextRecord() ) == NULL ) {
    
    // Nope. Create a new data chunk and append it to the end
    if ( ( newChunk = new DLDataChunk ( recordLength ) ) == NULL )
      MMFatalError( "DataLogger::initialize" , "Cannot allocate memory!" );
    curChunk->append ( newChunk );

    curChunk = newChunk;

    // Retrieve the pointer from the newly created chunk
    fill = curChunk->nextRecord();
  }

  // Call the function to fill in the data block
  fillRecord ( fill );

}

// DataLogger::export : Writes the contents of the chunks into a file ---------
void DataLogger::exportToFile ( char *basefilename, DL_OUTPUT_TYPE type, 
                                char **names ) {

  FILE          *outfile;
  FILE          *script;
  DLDataChunk   *cur;
  int           i;
  char          filename[128];
  char          scriptname[128];

  if ( basefilename == NULL ) {
    MMWarning( "DataLogger::exportToFile", "NULL base filename!" );
    return;
  }

  if ( firstChunk == NULL ) {
    MMWarning( "DataLogger::exportToFile", "Nothing to save!" );
    return;
  }

  strcpy( filename, basefilename );
  strcat( filename, ".data" ); 

  if ( ( outfile = fopen( filename, "wb" ) ) == NULL )
    return;

  if( type == DL_ASCII ){

    if( names ) { //create the script for matlab

      strcpy( scriptname, basefilename );
      strcat( scriptname, ".m" );

      if( ( script = fopen( scriptname, "wa" ) ) == NULL ) {

        fclose( outfile );
        return;
      }

      fprintf( script, "vars = load('%s');\n", filename );

      for( i = 0; i < recordLength; i++)

        fprintf( script,"%s = vars(:,%d);\n", names[i], i+1 );

      fclose( script );

    }//end if names

    // Start from the first chunk and loop over all chunks
    cur = firstChunk;

    while ( cur != NULL) {

      // Dump the contents of the chunk to the file
      cur->dumpToFile( outfile );

      // Proceed with the next chunk
      cur = cur->nextChunk();
    }

    fclose( outfile );

  } else 
    MMFatalError( "DataLogger::exportToFile", 
                  "MATLAB output type currently not supported!");
  return;

} //end exportToFile function

//DataLogger::exportViaNet- lets you output to file over FTP ----------------

int DataLogger::exportFileViaNet ( char *filename ) {

  FILE *fscript;
  FILE *out;
  char uname[128];
  char hostname[128];
  char pass[128];
 
  char path[128];
  char t1[128];
 
  //exportToFile ( basefilename, type, names);
  if( ( fscript = fopen( "SCRIPT.FTP" , "w" ) ) == NULL ) {
    return 1;
  }

  MMGetStringSymbol( "rhexftphost", hostname, "" );
  if( hostname[0] == '\0' ) MMFatalError( "DataLogger::exportFileViaNet", 
                                         "No hostname supplied for ftp" );
  MMGetStringSymbol( "rhexftpuser", uname, "" );
  if( uname[0] == '\0' ) MMFatalError( "DataLogger::exportFileViaNet", 
                                       "No username supplied for ftp" );

  MMGetStringSymbol( "rhexftppasswd", pass, "" );
  if( pass[0] == '\0' ) MMFatalError( "DataLogger::exportFileViaNet", 
                                      "No password supplied for ftp" );

  MMGetStringSymbol( "rhexftppath", path, "" );
  if( path[0] == '\0' ) MMFatalError( "DataLogger::exportFileViaNet", 
                                      "No path supplied for ftp" );

  fprintf( fscript, "open %s\n user %s %s\n bi \n cd %s\nput %s\n bye\n",
           hostname, uname, pass, path, filename );

  fclose( fscript );
  
  if ( system( "ftp -n < SCRIPT.FTP > ftp.out" ) == -1 )
    return -1;

  if ( ( out  = fopen( "ftp.out" , "r" ) ) == NULL ) {
    return -1;
  }

  while ( !feof( out ) ) {
             
    fscanf( out, "%s", t1 );
    if ( strcmp( "Not", t1 ) == 0 || strcmp( "failed", t1 ) == 0 
         || strcmp( "No", t1 ) == 0 ) {

        //  printf("ERROR\n");
        fclose( out );
        return -1; 
    }
  }
  fclose( out );

  //write script to file
  return 0;
}

int  DataLogger::removeFile (char *filename) {

  char temp[128];

  strcpy( temp, "rm -f " );
  strcat( temp, filename );
  printf( "%s\n", temp );
  if( system( temp ) == -1 ) 
    return -1;
  else
    return 0;
}

int  DataLogger::exportViaNet ( char *basefilename, DL_OUTPUT_TYPE type, 
                                char **names ) {
  char mfile[128];
  char dat[128];

  strcpy( mfile, basefilename );
  strcat( mfile, ".m" );
  strcpy( dat, basefilename );
  strcat( dat, ".data" );
  exportToFile( basefilename, type, names );
  if ( exportFileViaNet( mfile ) == -1)
    return -1;
  if ( exportFileViaNet( dat ) == -1)
    return -1;
  if ( removeFile( mfile ) == -1)
    return -1;
  if ( removeFile( dat ) == -1)
    return -1;
  return 0;
 //Joel- fill in function here! =) 

}

void DataLogger::reset( void ) {

  // Erase the old data
  erase();

  // Recreate the data structures
  initialize( recordLength );
  
}

void DataLogger::reset( int new_reclen ) {

  // Erase the old data
  erase();

  // Recreate the data structures
  initialize( new_reclen );
  
}
