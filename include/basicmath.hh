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
 * $Id: basicmath.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Basic math operators
 *
 * Created       : Haldun Komsuoglu, 01/30/2001
 * Last Modified : Haldun Komsuoglu, 01/30/2001
 *
 ********************************************************************/

#ifndef _BASICMATH_HH
#define _BASICMATH_HH

#include <math.h>
#include "types.hh"

// Saturation function: limits a given input in an interval [dw_bnd, up_bnd]
inline double saturate ( const double x, const double dw_bnd, 
                         const double up_bnd ) {
  if      (x > up_bnd)   return up_bnd;
  else if (x < dw_bnd)   return dw_bnd;
  return x;
};


// Signum function
inline double sign ( const double x ) { 
  return (x < 0) ? -1.0 : ((x > 0.0) ? 1 : 0.0);
};

// min and max functions
inline double fmax( double a, double b ) { return ( a > b )?a:b; };
inline double fmin( double a, double b ) { return ( a < b )?a:b; };

// Approximate equality
inline bool approx_equal( double x, double y, double tol ) {
  return bool( ( x >= y - tol ) && ( x <= y + tol ) );
};

#endif
