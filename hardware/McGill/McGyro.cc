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
 * $Id: McGyro.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Low level hardware interface to the gyros of the McGill
 * version of RHex
 *
 * Created       : Dave McMordie, 05/29/2001
 * Last Modified : Dave McMordie, 05/30/2001
 *
 ********************************************************************/

#include <math.h>
#include "ModuleManager.hh"
#include "McGillHW.hh"
#include "McComponents.hh"

#define G1_ADC_CHAN 10
#define G2_ADC_CHAN 11
#define G3_ADC_CHAN 12

// The TouchdownDetector module -------------------------------
#define GYRO_NAME      "gyrointegrator"
#define GYRO_ORDER     5000
#define GYRO_PERIOD    1
#define GYRO_OFFSET    0


//  Integrator module to keep track of angles:
class GyroIntegrator : public Module {

public:
    GyroIntegrator ( McGyro *gyr ) 
        : Module( GYRO_NAME, 0, false, false )
        {GyroObject = gyr;};

    void  init ( void ) { };
    void  uninit ( void ) { };
    void  activate ( void ) { };
    void  deactivate ( void ) {  };
    void  update ( void ) {GyroObject->update();};
private:
    McGyro *GyroObject;
};

//Constructor
McGyro::McGyro( Hardware *hw_in )
{

  int i;
  float gainD[3] = { 200.0*M_PI/180.0, 200.0*M_PI/180.0, 200.0*M_PI/180.0, };
  float offsetD[3] = { 2.50, 2.50, 2.50};
  Floats temp;
  char msg[128];
  GyroIntegrator *gyr;

  hw = hw_in;

  if ( hw == NULL || hw->analogIO == NULL )
    MMFatalError ( "McGyro::McGyro", "Invalid hardware object!" );
  
  MMMessage( "Initializing Gyro components..." );

  for (i=0;i<3;i++) oldRates[i] = rates[i] = angles[i]=0.0;
  lastTime = CLOCK_TO_SEC( MMReadClock( ));
  
  // Read the affine hyro map parameters from the main symbol table
  temp = MMGetArraySymbol( "gyro_voltage_offsets" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    offsets[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    offsets[ i ] = offsetD[i];

  temp = MMGetArraySymbol( "gyro_voltage_gains" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    gains[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    gains[ i ] = gainD[ i ];
  
  sprintf( msg, "\n  gx:{%1.2f,%1.2f}, gy:{%1.2f,%1.2f}, gz:{%1.2f,%1.2f}\n", 
           offsets[0], gains[0], offsets[1], gains[1], offsets[2], gains[2]);
  MMMessage( msg );
 
  gyr = new GyroIntegrator(this);
  
  MMAddModule ( gyr, GYRO_PERIOD, GYRO_OFFSET,
                GYRO_ORDER );
  MMActivateModule ( gyr );
   
}

// Destructor
McGyro::~McGyro( void ) {
}

/////////////////////////////////////////////////////////////////////
// McGyro::readRate -- Returns angular velocity about 'a' in rads/s
float McGyro::readRate( Axis a )
{
    switch (a)
    {
    case AXIS_X:
        return rates[0];

    case AXIS_Y:
        return rates[1];

    case AXIS_Z:
        return rates[2];

    default:
        return 0.0;
    }  
}
/////////////////////////////////////////////////////////////////////
// McGyro::readAngle -- Returns angular velocity about 'a' in rads/s
float McGyro::readAngle( Axis a )
{
    switch (a)
    {
    case AXIS_X:
        return angles[0];

    case AXIS_Y:
        return angles[1];

    case AXIS_Z:
        return angles[2];

    default:
        return 0.0;
  }  
}


//  Called by Module Manager every time period
void McGyro::update()
{
    int i;
    
    // Update rates
   rates[0] =  (hw->analogIO->read( G1_ADC_CHAN )
                -offsets[0])*gains[0];
   rates[1] =  (hw->analogIO->read( G2_ADC_CHAN )
                -offsets[1])*gains[1];   
   rates[2] =  (hw->analogIO->read( G3_ADC_CHAN )
                -offsets[2])*gains[2];
   
   // Integrate angles
   // Yer basic Euler integrator
   for(i=0;i<3;i++)
   {    
       angles[i] += 0.5*(oldRates[i]+rates[i])
           * ( CLOCK_TO_SEC(MMReadClock()) - lastTime);
       oldRates[i]=rates[i];
   }
   lastTime = CLOCK_TO_SEC(MMReadClock());

   return;
}

void McGyro::resetAngle(Axis a, float val)
{
    switch (a)
    {
    case AXIS_X:
        angles[0] = val;
        break;
    case AXIS_Y:
        angles[1] = val;
        break;
    case AXIS_Z:
        angles[2] = val;
        break;
    default:
        return;
    }   
}

void McGyro::setOffset(Axis a, float val)
{
    switch (a)
    {
    case AXIS_X:
        offsets[0] = val;
        break;
    case AXIS_Y:
        offsets[1] = val;
        break;
    case AXIS_Z:
        offsets[2] = val;
        break;
    default:
        return;
    }   
}


void McGyro::getInfo( Axis axis, float *resolution, float *limit ) {

  // The resolution is a function of the timer frequency and the
  // accelerometer range and period.

  // For 2G version
  *resolution = 43.0e-3;
  *limit = 2.0;
}
