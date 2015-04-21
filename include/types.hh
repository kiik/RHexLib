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
 * $Id: types.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The RHexLib data type definitions
 *
 * This file defines several data types which are not specific to any module
 *
 * Created       : Uluc Saranli, 11/16/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _TYPES_HH
#define _TYPES_HH

// Local includes ---------------------------------------
#include "vast.hh" // Defines a 64 bit integer data type 
#include "Strings.hh"
#include "Floats.hh"

// Type definitions --------------------------------------
// A Boolean type
#ifdef _LINUX_

typedef bool BOOL;

#endif

#ifdef _QNX4_

typedef enum{ false = 0, true = 1 } bool;
typedef bool BOOL;

#endif

// Shortcut types
typedef char int8;
typedef unsigned char uint8;
typedef short int int16;
typedef unsigned short int uint16;
typedef int int32;
typedef unsigned int uint32;
typedef vast int64;
typedef uvast uint64;
typedef unsigned int uint;

// The CLOCK datatype. Holds the clock value in us.
typedef int64 CLOCK;
#define CLOCK_TO_LONG(c) VAST_TO_LONG(c)
#define CLOCK_TO_DOUBLE(c) VAST_TO_DOUBLE(c)
#define SEC_TO_CLOCK( _num_ ) ( CLOCK ) ( 1000000.0 * (_num_) )
#define MIN_TO_CLOCK( _num_ ) ( CLOCK ) ( 60000000.0 * (_num_) )
#define HOUR_TO_CLOCK( _num_ ) ( CLOCK ) ( 360000000.0 * (_num_) )
#define CLOCK_TO_SEC( _num_ ) ( CLOCK_TO_DOUBLE( _num_ ) / 1000000.0 )
#define CLOCK_TO_MIN( _num_ ) ( CLOCK_TO_DOUBLE( _num_ ) / 60000000.0 )
#define CLOCK_TO_HOUR( _num_ ) ( CLOCK_TO_DOUBLE( _num_ ) / 360000000.0 )

#define MM_CLOCKS_PER_SEC 1000000L

// Some useful definitions
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#endif
