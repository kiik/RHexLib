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
 * $Id: RHexLogger.hh,v 1.7 2001/08/09 20:54:23 lmcwil Exp $
 *
 * This Module is a wrapper for the DataLogger class
 * 
 * This class reads in details about what is to be logged from a
 * configuration file and provides a generic type of DataLogger. It
 * works by using the .rc file to determine which of a fixed set of
 * variables will be logged, the name of a file to output to, and
 * possibly a hostname for ftp. The class also provides for up to 16
 * user-defined variables to be logged. In this case, the user must
 * supply to the constructor a pointer to a function which returns a
 * float and takes an int as its parameter.
 *
 * Created       : Laura McWilliams, 05/09/2001
 * Last Modified : Laura McWilliams, 08/01/2001
 *
 ********************************************************************/

//========================================================================
// CONFIGURING THE RHEXLOGGER IN THE RC FILE
// -----------------------------------------
//
//                          ACTUAL variable
// NAME in .rc file         (if you use standard global var names)
//----------------          -----------------------------------------
//MODULE RELATED VARIABLES-------------------------------------------
//time                      MMReadUTime()
//relative_time             MMReadUTime() - lastResetTime
//mod_enc_speed0            encreader[0]->getSpeed()
//mod_enc_speed1            encreader[1]->getSpeed()
//mod_enc_speed2            encreader[2]->getSpeed()
//mod_enc_speed3            encreader[3]->getSpeed()
//mod_enc_speed4            encreader[4]->getSpeed()
//mod_enc_speed5            encreader[5]->getSpeed()
//mod_enc_pos0              encreader[0]->getPosition()
//mod_enc_pos1              encreader[1]->getPosition()
//mod_enc_pos2              encreader[2]->getPosition()
//mod_enc_pos3              encreader[3]->getPosition()
//mod_enc_pos4              encreader[4]->getPosition()
//mod_enc_pos5              encreader[5]->getPosition()
//mod_analogout0            analogout[0]->getValue()
//mod_analogout1            analogout[1]->getValue()
//mod_analogout2            analogout[2]->getValue()
//mod_analogout3            analogout[3]->getValue()
//mod_analogout4            analogout[4]->getValue()
//mod_analogout5            analogout[5]->getValue()
//mod_poscontrol_pos0       positioncontrol[0]->getTarget().pos
//mod_poscontrol_pos1       positioncontrol[1]->getTarget().pos
//mod_poscontrol_pos2       positioncontrol[2]->getTarget().pos
//mod_poscontrol_pos3       positioncontrol[3]->getTarget().pos
//mod_poscontrol_pos4       positioncontrol[4]->getTarget().pos
//mod_poscontrol_pos5       positioncontrol[5]->getTarget().pos
//mod_poscontrol_vel0       positioncontrol[0]->getTarget().vel
//mod_poscontrol_vel1       positioncontrol[1]->getTarget().vel
//mod_poscontrol_vel2       positioncontrol[2]->getTarget().vel
//mod_poscontrol_vel3       positioncontrol[3]->getTarget().vel
//mod_poscontrol_vel4       positioncontrol[4]->getTarget().vel
//mod_poscontrol_vel5       positioncontrol[5]->getTarget().vel
//mod_spdfil0               speedfilter[0]->getSpeed()
//mod_spdfil1               speedfilter[1]->getSpeed()
//mod_spdfil2               speedfilter[2]->getSpeed()
//mod_spdfil3               speedfilter[3]->getSpeed()
//mod_spdfil4               speedfilter[4]->getSpeed()
//mod_spdfil5               speedfilter[5]->getSpeed()
//
//HARDWARE RELATED VARIABLES----------------------------------------
//hw_accel_x                hw->accels->read(AccelHW::AXIS_X)
//hw_accel_y                hw->accels->read(AccelHW::AXIS_Y)
//hw_accel_z                hw->accels->read(AccelHW::AXIS_Z)
//hw_gyro_rate_x            hw->gyros->readRate(GyroHW::AXIS_X)
//hw_gyro_rate_y            hw->gyros->readRate(GyroHW::AXIS_Y)
//hw_gyro_rate_z            hw->gyros->readRate(GyroHW::AXIS_Z)
//hw_gyro_angle_x           hw->gyros->readAngle(GyroHW::AXIS_X)
//hw_gyro_angle_y           hw->gyros->readAngle(GyroHW::AXIS_Y)
//hw_gyro_angle_z           hw->gyros->readAngle(GyroHW::AXIS_Z)
//hw_encoders0              hw->encoders->read(0)
//hw_encoders1              hw->encoders->read(1)
//hw_encoders2              hw->encoders->read(2)
//hw_encoders3              hw->encoders->read(3)
//hw_encoders4              hw->encoders->read(4)
//hw_encoders5              hw->encoders->read(5)
//hw_analogin0              hw->analogIO->read(0)
//hw_analogin1              hw->analogIO->read(1)
//hw_analogin2              hw->analogIO->read(2)
//hw_analogin3              hw->analogIO->read(3)
//hw_analogin4              hw->analogIO->read(4)
//hw_analogin5              hw->analogIO->read(5)
//hw_analogin6              hw->analogIO->read(6)
//hw_analogin7              hw->analogIO->read(7)
//hw_analogin8              hw->analogIO->read(8)
//hw_analogin9              hw->analogIO->read(9)
//hw_analogin10             hw->analogIO->read(10)
//hw_analogin11             hw->analogIO->read(11)
//hw_analogin12             hw->analogIO->read(12)
//hw_analogin13             hw->analogIO->read(13)
//hw_analogin14             hw->analogIO->read(14)
//hw_analogin15             hw->analogIO->read(15)
//hw_digitalio0             hw->digitalIO->getByte(0)
//hw_digitalio1             hw->digitalIO->getByte(1)
//hw_digitalio2             hw->digitalIO->getByte(2)
//hw_digitalio3             hw->digitalIO->getByte(3)
//hw_digitalio4             hw->digitalIO->getByte(4)
//hw_digitalio5             hw->digitalIO->getByte(5)
//hw_digitalio6             hw->digitalIO->getByte(6)
//hw_timers0                hw->timers->read(0)
//hw_timers1                hw->timers->read(1)
//hw_timers2                hw->timers->read(2)
//hw_timers3                hw->timers->read(3)
//hw_timers4                hw->timers->read(4)
//hw_timers5                hw->timers->read(5)
//hw_timers6                hw->timers->read(6)
//hw_timers7                hw->timers->read(7)
//hw_timers8                hw->timers->read(8)
//hw_switches0              hw->switches->read(0)
//hw_switches1              hw->switches->read(1)
//hw_switches2              hw->switches->read(2)
//hw_switches3              hw->switches->read(3)
//hw_switches4              hw->switches->read(4)
//hw_switches5              hw->switches->read(5)
//hw_switches6              hw->switches->read(6)
//hw_switches7              hw->switches->read(7)
//hw_dials0                 hw->dials->read(0)
//hw_dials1                 hw->dials->read(1)
//hw_dials2                 hw->dials->read(2)
//hw_dials3                 hw->dials->read(3)
//hw_dials4                 hw->dials->read(4)
//hw_dials5                 hw->dials->read(5)
//hw_dcmotor_volt0          hw->dcmotors->getVoltage(0)
//hw_dcmotor_volt1          hw->dcmotors->getVoltage(1)
//hw_dcmotor_volt2          hw->dcmotors->getVoltage(2)
//hw_dcmotor_volt3          hw->dcmotors->getVoltage(3)
//hw_dcmotor_volt4          hw->dcmotors->getVoltage(4)
//hw_dcmotor_volt5          hw->dcmotors->getVoltage(5)
//hw_dcmotor_curr0          hw->dcmotors->getCurrent(0)
//hw_dcmotor_curr1          hw->dcmotors->getCurrent(1)
//hw_dcmotor_curr2          hw->dcmotors->getCurrent(2)
//hw_dcmotor_curr3          hw->dcmotors->getCurrent(3)
//hw_dcmotor_curr4          hw->dcmotors->getCurrent(4)
//hw_dcmotor_curr5          hw->dcmotors->getCurrent(5)
//hw_dcmotor_backemf0       hw->dcmotors->getBackEMF(0)
//hw_dcmotor_backemf1       hw->dcmotors->getBackEMF(1)
//hw_dcmotor_backemf2       hw->dcmotors->getBackEMF(2)
//hw_dcmotor_backemf3       hw->dcmotors->getBackEMF(3)
//hw_dcmotor_backemf4       hw->dcmotors->getBackEMF(4)
//hw_dcmotor_backemf5       hw->dcmotors->getBackEMF(5)
//hw_dcmotor_temp0          hw->dcmotors->getTemperature(0)
//hw_dcmotor_temp1          hw->dcmotors->getTemperature(1)
//hw_dcmotor_temp2          hw->dcmotors->getTemperature(2)
//hw_dcmotor_temp3          hw->dcmotors->getTemperature(3)
//hw_dcmotor_temp4          hw->dcmotors->getTemperature(4)
//hw_dcmotor_temp5          hw->dcmotors->getTemperature(5)
//hw_power_voltage          hw->power->voltage()
//hw_power_current          hw->power->current()
//
//USER-DEFINED VARIABLES---------------------------------------------
//user_defined0             rhexLoggerGlobalVars[0]
//user_defined1             rhexLoggerGlobalVars[1]
//user_defined2             rhexLoggerGlobalVars[2]
//user_defined3             rhexLoggerGlobalVars[3]
//user_defined4             rhexLoggerGlobalVars[4]
//user_defined5             rhexLoggerGlobalVars[5]
//user_defined6             rhexLoggerGlobalVars[6]
//user_defined7             rhexLoggerGlobalVars[7]
//user_defined8             rhexLoggerGlobalVars[8]
//user_defined9             rhexLoggerGlobalVars[9]
//user_defined10            rhexLoggerGlobalVars[10]
//user_defined11            rhexLoggerGlobalVars[11]
//user_defined12            rhexLoggerGlobalVars[12]
//user_defined13            rhexLoggerGlobalVars[13]
//user_defined14            rhexLoggerGlobalVars[14]
//user_defined15            rhexLoggerGlobalVars[15]
//----------------          -----------------------------------------
//
//FORMAT of .rc file-
//
// rhexloggerbasefile = "base_outfile_name";*
// rhexloggervars = { "1st_var_name" "2nd_var_name"..."last_var_name" };
// logging_period = some_decimal;
//
// rhexftphost   = "name_of_ftp_host"; # you need all the following
// rhexftpuser   = "ftp_username";     # to do ftp export
// rhexftppasswd = "ftp_password";
// rhexftppath   = "ftp_path";
//
//*NOTE: you can include %i, %d and/or %t anywhere in the string itself 
//       to have the logger automatically insert into the filename the
//       index of the logging session, date, and time, respectively, with
//       the format:
//       index:     iiii
//       date:      yymmdd
//       time:      hhmmss
//
// CONFIGURING THE RHEXLOGGER AFTER CONSTRUCTION
// ---------------------------------------------
//
// 2 functions can be used to augment RHexLogger's functionality after
// it has already been constructed. These functions must be called
// before the module has been activated, but may be called after
// initialization.
//
// bool RHexLogger::addNewFunc( char* name, logFnPtr func, int index );
// bool RHexLogger::addNewVar( char* name, float* variable );
//
// The first option allows you to pass it a function pointer to a
// function that has the format float func( int index ). The value of
// this variable will be determined by calling func with whatever
// index is supplied. The second option allows you to directly access
// a float variable via its address. It is up to the user to ensure
// that the address will always point to a real value of type float. I
// think this is kind of unsafe, but hopefully it will get Dave off my
// back.
//  
// Both functions return TRUE if there is an error and the
// variable or function cannot be logged. However, in practice this
// will never happen because in the style of Uluc I have placed
// FatalErrors in all such cases.
//========================================================================


#ifndef _RHEXLOGGER_HH
#define _RHEXLOGGER_HH

#include <stdio.h>
#include "ModuleManager.hh"
#include "types.hh"
#include "DataLogger.hh"

#define MAX_ENTRIES 140 // this is the maximum number of variables 
                        // you are allowed to log

// Global variables ----------------------------------------------------------

extern float rhexLoggerGlobalVars[16];



// ===========================================================================
// Prototype declarations

class RHexLogger;
class GenericLogger;

// ===========================================================================
// GenericLogger class

class GenericLogger : public DataLogger {

public:

// Constructors-------------------------------------------------------
  GenericLogger( RHexLogger* owner_in, int reclen_in ) 
    : DataLogger( reclen_in, "genericlogger" ),
      owner( owner_in ),
      reclen( reclen_in ) { } 

  void fillRecord( float *f );

private:

  RHexLogger * owner;
  int reclen;

};// end of GenericLogger class

// ============================================================================
// RHexLogger class

class RHexLogger : public Module {

public:

// data structures -----------------------------------------------------------

  typedef float( * logFnPtr )( uint index ); //function pointer type

  typedef struct { //table entry or array entry in RHexLogger class

    char * handle;    // nickname for this variable
    logFnPtr myfunc;
    uint index;

    float getVal( void ){ return( myfunc( index ) ); }

  } LogElt;

// constructors --------------------------------------------------------------

  RHexLogger( void );
  ~RHexLogger() { delete logger; }

  //Module functions
  void init( void );
  void uninit( void );
  void activate( void );
  void deactivate( void );
  void update( void ) { }
  
  int exportToFile( DL_OUTPUT_TYPE type = DL_ASCII );
  int exportViaNet( DL_OUTPUT_TYPE type = DL_ASCII );

  void addNewFunc( char* name, logFnPtr func, int index );
  void addNewVar( char* name, float* variable );

  void reset( void );
  void setLoggingPeriod( int period ){ logger->setLoggingPeriod( period ); }

  char*   getLastFilename( void ) { return thisfilename; };
  Strings getVariables( void );

  void getLogElts( Strings varNames, LogElt array[] );
  float getVariable( char* name );

  // retained for backwards-compatibility
  Strings get_variables( void ) { return getVariables(); }
  char* get_thisfilename( void ) { return thisfilename; };
  float getAddedVar( uint index ) { return * (vars[index]); };

  void printInfo( void );

private:

  float  *vars[ MAX_ENTRIES ];  // array of addresses of variables
  int    num_vars;              // number elements in vars

  LogElt stuff[ MAX_ENTRIES ];  // array of logFnPtr structs 
  int    num_to_log;            // number of variables to log

  char   basefilename[128];     // name of outfile(s)
  GenericLogger * logger;          // underlying DataLogger object
  int    logging_index;         // index of current logging session
  char   thisfilename[128];     // the last base filename written to 

  // utility methods
  float  func_call( int i );       // used to perform actual updates
  char** make_names(void); 
  void   update_thisfilename( void );// updates filename before an export    
  bool   check_basefilename( void ); // checks if filename is valid

  friend class GenericLogger;

};//end of RHexLogger class


#endif

  
