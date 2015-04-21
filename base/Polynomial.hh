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
 * $Id: Polynomial.hh,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Polynomial representation class 
 *
 *  Contains:
 *      - Polynomial class
 *      - PolynomialRecord structure (to define arrays of polynomials)
 *
 * Created       : Haldun Komsuoglu, 03/21/2000
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#ifndef _POLYNOMIAL_HH
#define _POLYNOMIAL_HH

#include "Buffer.hh"

/*---------------------------------------------------------------------*/
/* The Polynomial class -----------------------------------------------*/
/*---------------------------------------------------------------------*/
class Polynomial : public Buffer {

 private:
  
  // Polynomial power sign. 
  // if PowSign = -1 => P(x) = a0 + a1 x^{-1} + ... + aN x^{-M}
  // if PowSign = +1 => P(x) = a0 + a1 x^{1} + ... + aN x^{M}, default
  int      PowSign;

 public:

  // Default constructor -- constant positive powered polynomial w/ a0 = 0
  Polynomial( void );
  // Constant polynomial constructor (conversion law) with a given 
  // constant w/ positive power convension
  Polynomial( double );
  // Buffer constructor with element value initialization (assumes positive
  // power sign)
  Polynomial( double[], int);
  // Buffer constructor with element value initialization, and power sign
  // assignment
  Polynomial( double[], int, int);


  // Returns the order of the polynomial
  int Order() const {return (size()-1);};

  // Sets the polynomial order sign interperation
  void setPowSign( int Sign ) { PowSign = Sign; };

  // Gets the polynomial order sign interperation
  int getPowSign( void ) const { return PowSign; };

  // Returns the pointer to the begining of the coefficient
  // list of the pointer
  double *CoeffList() { return Head; };

  // Polynomial evaluation at a given value
  double operator()( double );

  // Polynomial assignment / equation operator
  void operator=( const Polynomial& );
  
  // Polynomial multiplication/assignment operation
  Polynomial& operator*=( const Polynomial& );

  // Polynomial addition/assiginment operation
  Polynomial& operator+=( const Polynomial& );

  // Polynomial subtraction/assignment operation
  Polynomial& operator-=( const Polynomial& );

  // Polynomial substitution/assignment operation
  Polynomial& operator<<=( const Polynomial& );

  // Displays (prints out) the polynomial in a formated form in the
  // standard output with the specified polynomial name
  void display( char* );

  // Displays (prints out) the polynomial in a formated form in the
  // standard output
  void display( void ) { display( "P(x)" ); }; 


};




#endif

