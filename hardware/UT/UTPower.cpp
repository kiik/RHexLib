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

/** @file UTPower.cpp
 *  @brief Low level hardware interface to the power measurement facilities the Tartu University version of RHex
 *
 *  @author Meelik Kiik
 *  @version 0.1
 *  @date 21/04/2015 16:00
 *
 *  Created at 21/04/2015 16:00
 */

#include <math.h>
#include "UTComponents.h"

UTPower::UTPower( Hardware *hw ) {

  Floats temp;
  char msg[128];

  MMMessage( "Initializing power components..." );

  // Latest measured calibration:
  // v = { 1.9792, 5.6557 }
  // i = { 0, 4.0860 }

  // Read the affine accelerometer map parameters from the main symbol
  // table Note that the defaults are 0.0 for easy debugging
  temp = MMGetArraySymbol( "batt_voltage_calib" );
  vOffset = ( temp.getCount() > 0 ) ? temp.get(0) : 0.0;
  vScale = ( temp.getCount() > 1 ) ? temp.get(1) : 0.0;

  temp = MMGetArraySymbol( "batt_current_calib" );
  iOffset = ( temp.getCount() > 0 ) ? temp.get(0) : 0.0;
  iScale = ( temp.getCount() > 1 ) ? temp.get(1) : 0.0;

  sprintf( msg, "v:{%1.2f,%1.2f}, i:{%1.2f,%1.2f}.", 
           vOffset, vScale, iOffset, iScale );
  MMMessage( msg );

  analogIO = hw->analogIO;

  MMMessage( "done.\n" );

}
