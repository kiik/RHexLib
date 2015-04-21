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
 * $Id: RHexLogger.cc,v 1.9 2001/08/09 20:54:23 lmcwil Exp $
 *
 * This file provides implementation for RHexLogger.hh
 *
 * Created       : Laura McWilliams, 05/09/2001
 * Last Modified : Laura McWilliams, 08/01/2001
 *
 ********************************************************************/

#include <time.h>
#include <string>

#include "ModuleManager.hh"
#include "Hardware.hh"
#include "DataLogger.hh"
#include "StdModules.hh"
#include "types.hh"
#include "AnalogOutput.hh"
#include "EncoderReader.hh"
#include "PositionControl.hh"
#include "MotorControl.hh"
#include "SpeedFilter.hh"
#include "RHexLogger.hh"

static Hardware        * hw_ptr;
static EncoderReader   * enc_ptrs[6];
static AnalogOutput    * ana_ptrs[6];
static PositionControl * poscon_ptrs[6];
static SpeedFilter     * spdfil_ptrs[6];

// Last time the logger was reset.
static double            resetMark = 0;

// Makes sure that only one instance is created.
static bool       creationStatus = false;
static RHexLogger *myInstance = NULL;

//optional global variables to use for logging
float rhexLoggerGlobalVars[16];

// Wrapper functions for various variables in the system ----------------
// MODULE-RELATED-

static float getEncPos( uint index ) { return enc_ptrs[index]->getPosition(); }

static float getEncSpeed( uint index ) { return enc_ptrs[index]->getSpeed(); }

static float getTime( uint index ) { 
  if ( index == 0 )
    return MMReadUTime(); 
  else
    return ( MMReadUTime() - resetMark ); 
}

static float getAnalogOut( uint index ) { return ana_ptrs[index]->getValue(); }

static float getTargetPos( uint index ) { 

  MotorTarget_t tmp;

  poscon_ptrs[index]->getTarget( &tmp );

  return tmp.pos;
}

static float getTargetVel( uint index ) {

  MotorTarget_t tmp;

  poscon_ptrs[index]->getTarget( &tmp );

  return tmp.vel;
}

static float getSpdFilSpeed( uint index ) { return spdfil_ptrs[index]->getSpeed(); }

//HARDWARE-RELATED-
static float getAccel( uint index ) {

  char msg[128];

  if( index == 0 ) return ( hw_ptr->accels )->read( AccelHW::AXIS_X );
  else if ( index == 1 ) return ( hw_ptr->accels )->read( AccelHW::AXIS_Y );
  else if ( index == 2 ) return ( hw_ptr->accels )->read( AccelHW::AXIS_Z );
  else {
    sprintf( msg,  "Incorrect index supplied: %d\n", index );
    MMFatalError( "getAccel", msg );
    return 0.0;

  }
}

static float getHWGyroRate( uint index ) {

  char msg[128];

  if( index == 0 ) return ( hw_ptr->gyros )->readRate( GyroHW::AXIS_X );
  else if ( index == 1 ) return ( hw_ptr->gyros )->readRate( GyroHW::AXIS_Y );
  else if ( index == 2 ) return ( hw_ptr->gyros )->readRate( GyroHW::AXIS_Z );
  else {
    sprintf( msg,  "Incorrect index supplied: %d\n", index );
    MMFatalError( "getHWGyroRate", msg );
    return 0.0;

  }
}

static float getHWGyroAngle( uint index ) {

  char msg[128];

  if( index == 0 ) return ( hw_ptr->gyros )->readAngle( GyroHW::AXIS_X );
  else if ( index == 1 ) return ( hw_ptr->gyros )->readAngle( GyroHW::AXIS_Y );
  else if ( index == 2 ) return ( hw_ptr->gyros )->readAngle( GyroHW::AXIS_Z );
  else {
    sprintf( msg,  "Incorrect index supplied: %d\n", index );
    MMFatalError( "getHWGyroAngle", msg );
    return 0.0;

  }
}

static float getHWEnc( uint index ) { 
  return ( hw_ptr->encoders )->read( index ); 
}

static float getHWanalogInput( uint index ) { 
  return ( hw_ptr->analogIO )->read( index ); 
}

static float getHWdigitalIO( uint index ) { 
  return ( hw_ptr->digitalIO )->getByte( index ); 
}

static float getHWtimers( uint index ) { 
  return ( hw_ptr->timers )->read( index ); 
}

static float getHWswitches( uint index ) { 
  return ( hw_ptr->switches )->read( index ); 
}

static float getHWdials( uint index ){ 
  return ( hw_ptr->dials )->read( index ); 
}

static float getHWdcmVolt( uint index ) { 
  return ( hw_ptr->dcmotors )->getVoltage( index ); 
}

static float getHWdcmCurr( uint index ) { 
  return ( hw_ptr->dcmotors )->getCurrent( index ); 
}

static float getHWdcmBackEMF( uint index ) { 
  return ( hw_ptr->dcmotors )->getBackEMF( index ); 
}

static float getHWdcmTemp( uint index ) { 
  return ( hw_ptr->dcmotors )->getTemperature( index ); 
}

static float getHWvoltage( uint index ) {
  return ( hw_ptr->power )->voltage();
}

static float getHWcurrent( uint index ) {
  return ( hw_ptr->power )->current();
}

static float getGlobalVar( uint index )  { 

  if ( index < 16 )
    return rhexLoggerGlobalVars[index];
  else
    MMFatalError( "getGlobalVar", "There are only 16 global logging variables!" );

  return 0.0;
}


// Returns a variable which is added by the user.
static float getAddedVar( uint index )  { 

  if ( myInstance == NULL ) 
    MMFatalError( "getAddedVar", "You need to create a RHexLogger instance!" );

  return myInstance->getAddedVar(index);
}

//---------------------------------------------------------------------------


//===========================================================================
//define a "table" mapping variable names to function pointers and 
//indices with which to call those functions.

static int table_size = MAX_ENTRIES;  //(currently 139)

static RHexLogger::LogElt table[ MAX_ENTRIES ] = {
  { "time", getTime, 0 },
  { "relative_time", getTime, 1 },
  { "hw_accel_x", getAccel, 0 },
  { "hw_accel_y", getAccel, 1 },
  { "hw_accel_z", getAccel, 2 },
  { "hw_gyro_rate_x", getHWGyroRate, 0 },
  { "hw_gyro_rate_y", getHWGyroRate, 1 },
  { "hw_gyro_rate_z", getHWGyroRate, 2 },
  { "hw_gyro_angle_x", getHWGyroAngle, 0 },
  { "hw_gyro_angle_y", getHWGyroAngle, 1 },
  { "hw_gyro_angle_z", getHWGyroAngle, 2 },
  { "mod_enc_speed0", getEncSpeed, 0 },
  { "mod_enc_speed1", getEncSpeed, 1 },
  { "mod_enc_speed2", getEncSpeed, 2 },
  { "mod_enc_speed3", getEncSpeed, 3 },
  { "mod_enc_speed4", getEncSpeed, 4 },
  { "mod_enc_speed5", getEncSpeed, 5 },
  { "mod_enc_pos0", getEncPos, 0 },
  { "mod_enc_pos1", getEncPos, 1 },
  { "mod_enc_pos2", getEncPos, 2 },
  { "mod_enc_pos3", getEncPos, 3 },
  { "mod_enc_pos4", getEncPos, 4 },
  { "mod_enc_pos5", getEncPos, 5 },
  { "mod_analogout0", getAnalogOut, 0 },
  { "mod_analogout1", getAnalogOut, 1 },
  { "mod_analogout2", getAnalogOut, 2 },
  { "mod_analogout3", getAnalogOut, 3 },
  { "mod_analogout4", getAnalogOut, 4 },
  { "mod_analogout5", getAnalogOut, 5 },
  { "mod_poscontrol_pos0", getTargetPos, 0 },
  { "mod_poscontrol_pos1", getTargetPos, 1 },
  { "mod_poscontrol_pos2", getTargetPos, 2 },
  { "mod_poscontrol_pos3", getTargetPos, 3 },
  { "mod_poscontrol_pos4", getTargetPos, 4 },
  { "mod_poscontrol_pos5", getTargetPos, 5 },
  { "mod_poscontrol_vel0", getTargetVel, 0 },
  { "mod_poscontrol_vel1", getTargetVel, 1 },
  { "mod_poscontrol_vel2", getTargetVel, 2 },
  { "mod_poscontrol_vel3", getTargetVel, 3 },
  { "mod_poscontrol_vel4", getTargetVel, 4 },
  { "mod_poscontrol_vel5", getTargetVel, 5 },
  { "mod_spdfil0", getSpdFilSpeed, 0 },
  { "mod_spdfil1", getSpdFilSpeed, 1 },
  { "mod_spdfil2", getSpdFilSpeed, 2 },
  { "mod_spdfil3", getSpdFilSpeed, 3 },
  { "mod_spdfil4", getSpdFilSpeed, 4 },
  { "mod_spdfil5", getSpdFilSpeed, 5 },
  { "hw_encoders0", getHWEnc, 0 },
  { "hw_encoders1", getHWEnc, 1 },
  { "hw_encoders2", getHWEnc, 2 },
  { "hw_encoders3", getHWEnc, 3 },
  { "hw_encoders4", getHWEnc, 4 },
  { "hw_encoders5", getHWEnc, 5 },
  { "hw_analogin0", getHWanalogInput, 0 },
  { "hw_analogin1", getHWanalogInput, 1 },
  { "hw_analogin2", getHWanalogInput, 2 },
  { "hw_analogin3", getHWanalogInput, 3 },
  { "hw_analogin4", getHWanalogInput, 4 },
  { "hw_analogin5", getHWanalogInput, 5 },
  { "hw_analogin6", getHWanalogInput, 6 },
  { "hw_analogin7", getHWanalogInput, 7 },
  { "hw_analogin8", getHWanalogInput, 8 },
  { "hw_analogin9", getHWanalogInput, 9 },
  { "hw_analogin10", getHWanalogInput, 10 },
  { "hw_analogin11", getHWanalogInput, 11 },
  { "hw_analogin12", getHWanalogInput, 12 },
  { "hw_analogin13", getHWanalogInput, 13 },
  { "hw_analogin14", getHWanalogInput, 14 },
  { "hw_analogin15", getHWanalogInput, 15 },
  { "hw_digitalio0", getHWdigitalIO, 0 },
  { "hw_digitalio1", getHWdigitalIO, 1 },
  { "hw_digitalio2", getHWdigitalIO, 2 },
  { "hw_digitalio3", getHWdigitalIO, 3 },
  { "hw_digitalio4", getHWdigitalIO, 4 },
  { "hw_digitalio5", getHWdigitalIO, 5 },
  { "hw_timers0", getHWtimers, 0 },
  { "hw_timers1", getHWtimers, 1 },
  { "hw_timers2", getHWtimers, 2 },
  { "hw_timers3", getHWtimers, 3 },
  { "hw_timers4", getHWtimers, 4 },
  { "hw_timers5", getHWtimers, 5 },
  { "hw_timers6", getHWtimers, 6 },
  { "hw_timers7", getHWtimers, 7 },
  { "hw_timers8", getHWtimers, 8 },
  { "hw_switches0", getHWswitches, 0 },
  { "hw_switches1", getHWswitches, 1 },
  { "hw_switches2", getHWswitches, 2 },
  { "hw_switches3", getHWswitches, 3 },
  { "hw_switches4", getHWswitches, 4 },
  { "hw_switches5", getHWswitches, 5 },
  { "hw_switches6", getHWswitches, 6 },
  { "hw_switches7", getHWswitches, 7 },
  { "hw_dials0", getHWdials, 0 },
  { "hw_dials1", getHWdials, 1 },
  { "hw_dials2", getHWdials, 2 },
  { "hw_dials3", getHWdials, 3 },
  { "hw_dials4", getHWdials, 4 },
  { "hw_dials5", getHWdials, 5 },
  { "hw_dcmotor_volt0", getHWdcmVolt, 0 },
  { "hw_dcmotor_volt1", getHWdcmVolt, 1 },
  { "hw_dcmotor_volt2", getHWdcmVolt, 2 },
  { "hw_dcmotor_volt3", getHWdcmVolt, 3 },
  { "hw_dcmotor_volt4", getHWdcmVolt, 4 },
  { "hw_dcmotor_volt5", getHWdcmVolt, 5 },
  { "hw_dcmotor_curr0", getHWdcmCurr, 0 },
  { "hw_dcmotor_curr1", getHWdcmCurr, 1 },
  { "hw_dcmotor_curr2", getHWdcmCurr, 2 },
  { "hw_dcmotor_curr3", getHWdcmCurr, 3 },
  { "hw_dcmotor_curr4", getHWdcmCurr, 4 },
  { "hw_dcmotor_curr5", getHWdcmCurr, 5 },
  { "hw_dcmotor_backemf0", getHWdcmBackEMF, 0 },
  { "hw_dcmotor_backemf1", getHWdcmBackEMF, 1 },
  { "hw_dcmotor_backemf2", getHWdcmBackEMF, 2 },
  { "hw_dcmotor_backemf3", getHWdcmBackEMF, 3 },
  { "hw_dcmotor_backemf4", getHWdcmBackEMF, 4 },
  { "hw_dcmotor_backemf5", getHWdcmBackEMF, 5 },
  { "hw_dcmotor_temp0", getHWdcmTemp, 0 },
  { "hw_dcmotor_temp1", getHWdcmTemp, 1 },
  { "hw_dcmotor_temp2", getHWdcmTemp, 2 },
  { "hw_dcmotor_temp3", getHWdcmTemp, 3 },
  { "hw_dcmotor_temp4", getHWdcmTemp, 4 },
  { "hw_dcmotor_temp5", getHWdcmTemp, 5 },
  { "hw_power_voltage", getHWvoltage, 0 },
  { "hw_power_current", getHWcurrent, 0 },
  { "user_defined0", getGlobalVar, 0 },
  { "user_defined1", getGlobalVar, 1 },
  { "user_defined2", getGlobalVar, 2 },
  { "user_defined3", getGlobalVar, 3 },
  { "user_defined4", getGlobalVar, 4 },
  { "user_defined5", getGlobalVar, 5 },
  { "user_defined6", getGlobalVar, 6 },
  { "user_defined7", getGlobalVar, 7 },
  { "user_defined8", getGlobalVar, 8 },
  { "user_defined9", getGlobalVar, 9 },
  { "user_defined10", getGlobalVar, 10 },
  { "user_defined11", getGlobalVar, 11 },
  { "user_defined12", getGlobalVar, 12 },
  { "user_defined13", getGlobalVar, 13 },
  { "user_defined14", getGlobalVar, 14 },
  { "user_defined15", getGlobalVar, 15 }
};

// lookup function-------------------------------------------------------------
// "looks up" the string to see if an entry in the array contains it.
// returns the index of the table, or -1 if not present

static int lookup( char* s ) {

  int i = 0;

  while ( ( i < table_size ) ) {

    if ( !strcmp( table[i].handle, s ) ) 
      return i; 

    i++;
  }

  return -1;

} // end lookup function


//===========================================================================

// GenericLogger::fillRecord-function-----------------------------------------
// fills in the record based on what's in the array of pointers to floats 
// in the owner RHexLogger

void GenericLogger::fillRecord( float * f ) {

  int i, num_to_log;

  num_to_log = owner->num_to_log;

  for( i = 0; i < num_to_log; i++ ) 
    f[i] = owner->func_call( i );


} // end-fillRecord-function---------------------------------------------------

//RHexLogger-ctor------------------------------------------------------------
//NOTE- this function assumes that the symbol table has been filled in 
//      allready in main. any variable not specified as TRUE in the config
//      file is not logged. the default outfile name is "rhex.out"
//sets up the RHexLogger instance according the config file.
//if given a function pointer parameter, it uses that as the function to get
//values of user-defined variables.
//---------------------------------------------------------------------------
RHexLogger::RHexLogger( void ) 
  : Module( RHEXLOGGER_NAME, 0, true, false ) {

  if ( creationStatus )
    MMFatalError( "RHexLogger::RHexLogger",
                  "Only one RHexLogger instance can be created!" );
  creationStatus = true;
  myInstance = this;
}

// end-of-ctors----------------------------------------------------------------


// RHexLogger::printInfo-------------------------------------------------------
// tells you what all values RHexLogger is set up to log
// a debug function

void RHexLogger::printInfo ( void ) {

  char msg[128];
  int i;

  sprintf( msg, "printInfo: num_to_log = %d \n", num_to_log ); 
  MMMessage( msg );
  MMMessage("these are the variables to be logged:\n");

  for( i = 0; i < num_to_log; i++ ) {
    sprintf( msg, "  %s\n", stuff[i].handle );
    MMMessage( msg );
  }

} // end printinfo function


// RHexLogger::init------------------------------------------------------------
void RHexLogger::init( void ) {

  int     i, temp, log_period;
  char    msg[128];
  Strings list;

  // initialize the static "global" pointers
  hw_ptr = MMGetHardware(); // find currently selected hardware

  for( i = 0; i < 6; i++ ){
    if( ( enc_ptrs[i] = ( EncoderReader *) 
          ( MMFindModule( "encreader", i )) ) == NULL )
      MMFatalError( "RHexLogger::init", "Cannot find EncoderReader" );
    if( ( ana_ptrs[i] = ( AnalogOutput *) 
          ( MMFindModule( "analogout", i )) ) == NULL )
      MMFatalError( "RHexLogger::init", "Cannot find AnalogOutput" );
    if( ( poscon_ptrs[i] = ( PositionControl *) 
          (MMFindModule( "poscontrol", i )) ) == NULL )
      MMFatalError( "RHexLogger::init", "Cannot find PositionControl" );
    if( ( spdfil_ptrs[i] = ( SpeedFilter *)
          ( MMFindModule( "spdfilter", i )) ) == NULL )
      MMFatalError( "RHexLogger::init", "Cannot find SpeedFilter" );
  } // end for

  // initialize some stuff
  logging_index = 0; 
  thisfilename[0] = '\0';

  // get the outfile name, check if it's valid
  MMGetStringSymbol( "rhexloggerbasefile", basefilename, "rhexlog" );

  if( check_basefilename() ) 
    MMFatalError( "RHexLogger:init", "Basefilename is invalid" );  

  // go through the table; figure out which values to log
  list = MMGetStrArraySymbol( "rhexloggervars" );

  if ( list.getCount() == 0 )
    MMWarning( "RHexLogger::RHexLogger", "rhexloggervars is not defined!" );

  num_to_log = 0;
  num_vars = 0;

  for( i = 0; i < list.getCount(); i++ ) {

    if( ( temp = lookup( list.get(i) ) ) >= 0 ) {

      stuff[ num_to_log ].handle = table[ temp ].handle;
      stuff[ num_to_log ].myfunc = table[ temp ].myfunc;
      stuff[ num_to_log ].index = table[ temp ].index;

      num_to_log++;

    } else {
      sprintf( msg, "Variable \"%s\" does not exist!", list.get( i ) );
      MMFatalError( "RHexLogger::RHexLogger", msg );
    }

  } // end for

  logger = NULL;
  // Create the underlying DataLogger module
  if( (logger = new GenericLogger( this, num_to_log )) == NULL)
    MMFatalError( "RHexLogger::activate", 
                  "new failed to allocate a new GenericLogger" );
  MMAddModule( logger, 1, 0, LOGGING_MODULES );

  // set its periodicity
  log_period = int( MMGetFloatSymbol( "logging_period", 1 ) ); 
  logger->setLoggingPeriod( log_period );

} // end init function

// RHexLogger::activate------------------------------------------------------
void RHexLogger::activate( void ) {

  MMGrabModule( logger, this );

  // Mark the base time for logging the time difference.
  resetMark = MMReadUTime();

} // end activate function

// RHexLogger::deactivate------------------------------------------------------
void RHexLogger::deactivate( void ) {

  // export(); //convenient for testing purposes

  MMReleaseModule( logger, this );

}//end deactivate function

// RHexLogger::uninit----------------------------------------------------------
// removes the logger module from the Module Manager

void RHexLogger::uninit( void ) {

  if ( logger ) MMRemoveModule( logger );
  if ( logger ) delete logger;
  logger = NULL;

} // end uninit function

//RHexLogger::func_call-------------------------------------------------------
//a private function to read the actual data.

inline float RHexLogger::func_call( int i ) {

  return stuff[i].myfunc( stuff[i].index );

}
    
// the-RHexLogger-export-functions---------------------------------------------
// export-to-file--------------------------------------------------------------
// returns non-zero on error.
int RHexLogger::exportToFile( DL_OUTPUT_TYPE type ) {

  char** array = make_names();

  this->update_thisfilename();

  //output the data
  logger->exportToFile( thisfilename, type, array );

  delete[] array;

  return 0;                   //??? need to implement error-checking
}

// export-via-net--------------------------------------------------------------
// returns non-zero on error
int RHexLogger::exportViaNet( DL_OUTPUT_TYPE type ) {

  char** array = make_names();

  update_thisfilename();

  //output the data
  int result = logger->exportViaNet( thisfilename, type, array );

  delete[] array;

  return result;
}

// reset---------------------------------------------------------------------
// Resets the logger contents and marks the time for time logging

void RHexLogger::reset( void ) { 

  logger->reset(); 
  logging_index++; 

  resetMark = MMReadUTime();

};

// make-names-----------------------------------------------------------------
// a private function used by the export functions

char ** RHexLogger::make_names( void ) {

  char** array;
  int    i;

  if( num_to_log == 0 ) 
    return NULL; // in case nothing is being logged

  // make array containing the names of the variables, in order of logging
  array = new char*[ num_to_log ];

  for( i = 0; i < num_to_log; i++ ) {

    array[i] = stuff[i].handle;

  }

  return array;
}

// update-thisfilename--------------------------------------------------------
// a private function called before exporting

void RHexLogger::update_thisfilename( void ) {

  int    i = 0;
  int    j = 0;
  char temp[128];

  time_t seconds = time( NULL );
  tm * now = localtime( &seconds );
  memset( thisfilename, 0, 128 * sizeof(char) );

  while( basefilename[i])
  { 
    while( basefilename[i] && ( basefilename[i] != '%' ) )
    { thisfilename[j++] = basefilename[i++]; }
    
    if( !basefilename[i++] ) break;

    memset( temp, 0, 128 * sizeof( char ) ); // reset temp string

    if( basefilename[i] == 'i') {            // use format iiii
      if( logging_index > 9999 ) logging_index %= 10000;
      sprintf( temp, "%04d", logging_index );
      strcat( thisfilename, temp );
    }// end if 'i'

    else if( basefilename[i] == 'd' ) {     // use format yyyymmdd
      sprintf( temp, "%04d%02d%02d", 
               (now->tm_year + 1900), (now->tm_mon + 1), now->tm_mday );
      strcat( thisfilename, temp );

    }// end if 'd'

    else if( basefilename[i] == 't' ) {    // use format hhmmss
      sprintf( temp, "%02d%02d%02d", now->tm_hour, now->tm_min, now->tm_sec );
      strcat( thisfilename, temp );
    }// end if 't'
  
    else {                                 // should never reach this point
      MMFatalError( "RHexLogger::update_thisfilename",
                         "Invalid character in basefilename" ); 
    }// end else-ifs

    i++;
    j = strlen( thisfilename );            // reset pointer to end of string

  }// end while
  return;
}// end-update-thisfilename ---------------------------------------------------
  
// check_basefilename-function ------------------------------------------------
// checks if the basefilename is ok, if not, returns nonzero if yes 0 otherwise
bool RHexLogger::check_basefilename( void ) {
  int i = 0;
  while( basefilename[i] ) 
  {  
    if( basefilename[i++] == '%' ) {
      if( basefilename[i] != 'd' 
          && basefilename[i] != 'i' 
          && basefilename[i] != 't' )
        return true;
      }// end if
    i++;
  }//end while
  return false;
}

// get_variables function-----------------------------------------------------
// returns a strings object containing the names of all variables being logged
Strings RHexLogger::getVariables( void ) {

  int     i;
  Strings blah( num_to_log );

  // fill the strings object with names of the variables, in order of logging
  for( i = 0; i < num_to_log; i++ ) {
    blah.set( i, stuff[i].handle );
  }

  return blah;

}// end get_variables function------------------------------------------------


//??? new function i just added to let you add more variables
void RHexLogger::addNewFunc( char* name, logFnPtr func, int index ){

  char msg[256];

  // check if module is already active
  if( this->getState() == Module::ACTIVE )
    MMFatalError( "RHexLogger::AddNewFunc", 
                  "Cannot add more variables after logger is activated!" );

  // check if maximum things already being logged
  if( !(num_to_log < MAX_ENTRIES) )
    MMFatalError( "RHexLogger::addNewFunc", 
                  "The maximum number of things is already being logged!" );

  stuff[ num_to_log ].handle = strdup( name );
  stuff[ num_to_log ].myfunc = func;
  stuff[ num_to_log ].index = index;

  num_to_log++;

  sprintf( msg,  "RHexLogger: Adding new variable %s and resetting logger\n", name );
  MMMessage( msg );

  logger->reset( num_to_log );
}


//???another new function to let you add more stuff
// RHexLogger::addNewVar----------------------------------------------------
void RHexLogger::addNewVar( char* name, float* variable ){

  vars[ num_vars ] = variable;
  addNewFunc( name, ( logFnPtr ) ::getAddedVar, num_vars );

  num_vars++;
  
}//end of addNewVar function



// getLogFnPtrs a new function i added for the
// InformationChannel to use to get function pointers so it can update
// its stuff
void RHexLogger::getLogElts( Strings varNames, LogElt array[] ){

  int i, temp;
  char msg[128];

  for( i = 0; i < varNames.getCount(); i++ ){

    if( ( temp = lookup( varNames.get(i) ) ) >= 0 ) {

      array[i].handle = table[ temp ].handle;
      array[i].myfunc = table[ temp ].myfunc;
      array[i].index = table[ temp ].index;

    } else {
      sprintf( msg, "Variable \"%s\" does not exist!", varNames.get( i ) );
      MMFatalError( "RHexLogger::getLogElts", msg );
    }
  }//end for

    return;
}// end of getLogFnPtrs function--------------------------------------


// another new function to allow you to check the value of a single
// variable one time
float RHexLogger::getVariable( char* name ){
  int i;
  char msg[128];

  if( (i = lookup( name )) >= 0 )
    return table[i].myfunc( table[i].index );
  else{
    sprintf( msg, "Variable \"%s\" does not exist!", name );
    MMFatalError( "RHexLogger::logOnce", msg );
  }
  return -1;
}// end of logOnce function----------------------------------------
