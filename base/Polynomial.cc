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
 * $Id: Polynomial.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Polynomial representation class 
 *
 * Created       : Haldun Komsuoglu, 03/21/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#include "Polynomial.hh"
#include <stdio.h>



/*----------------------------------------------------------------------*/
/*+++++++++++++++++++++++++ Polynomial Class +++++++++++++++++++++++++++*/
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::Polynomial()  : Default polynomial constructor for buffer. 
                               Resets all the memory locations and sets 
                               the polynomial to constant 0..
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial::Polynomial( void )
  : Buffer( 1 ) {

  PowSign = 1;   // Default power sign is positive

}
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::Polynomial( c ) : Polynomial constructor for constant
                                 polynomials with constant being c. 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial::Polynomial( double c )
  : Buffer( &c, 1 ) {

  PowSign = 1;   // Default power sign

}
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::Polynomial( a[], Order ) : Generic polynomial constructor 
                                          with coefficient value 
                                          initialization.

       
      a[]   : (double[]) The list of coefficient values to be initalized. 
              The order is a[] = {a0, a1, ..., aN} 
      Order : (int) Order of the polynomial 
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial::Polynomial( double a[],
                        int    Order )
   : Buffer ( a, Order+1 ){

  PowSign = 1;   // Default power sign

}
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::Polynomial( a[], Order, dPS ) : Generic polynomial 
                                               constructor with 
                                               coefficient value 
                                               initialization.

       
      a[]   : (double[]) The list of coefficient values to be initalized. 
              The order is a[] = {a0, a1, ..., aN} 
      Order : (int) Order of the polynomial 
      dPS   : (int) Desired power sign convention
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial::Polynomial( double a[],
                        int    Order,
                        int    dPS)
   : Buffer ( a, Order+1 ){

  PowSign = dPS;   // Default power sign

}
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::operator()( x_val ) : Evaluates a polynomial represented 
                                     by the coefficients stored in the 
                                     list at the given x value.

          return : (double) Resulting polynomial value, P(x)
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
double Polynomial::operator()( double x_val ){

  double   Total = 0;
  double   x_pow_i = 1;
  double   * a_i = Head;

  while (a_i <= End) {
    Total = Total + *a_i * x_pow_i;
    if (PowSign == 1)
      x_pow_i = x_pow_i * x_val;
    else
      x_pow_i = x_pow_i / x_val;
    a_i++;
  }
  
  return Total;
}
/*----------------------------------------------------------------------*/



/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::Extra_AssignmentOperator( pntr ) : Copies the data 
                                                  structures of Pequ
                                                  to "this".
          pntr   : (void*) The pointer to the right hand side operator
                   of the assignment operator operant
                    
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//void Polynomial::Extra_AssignmentOperator( void *pntr ) {
void Polynomial::operator=( const Polynomial& Pequ ) {

  // Copy the coefficient buffer
  Buffer::operator=( Pequ );

  // Copy the power sign convention
  PowSign = Pequ.getPowSign();

}
/*----------------------------------------------------------------------*/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::operator+=( Padd ) : Computes the sum of "this" polynomial
                                    and P2 and assigns it to "this" 
                                    polynomial.
          Padd   : (Polynomial&) The right side operant of the 
                   addition. Access via reference
                    
          return : (Polynomial&) Resulting polynomial, Psum = P + Padd,
                   Accessed via reference.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial& Polynomial::operator+=( const Polynomial& Padd ){

  Polynomial Pacc;
  int        i;
  double     *P1_ai;
  double     *P2_ai;

  // Error check: If non of the polynomials is a constant and the power
  // sign convension of these two polynomials do not match give an error
  // message and exit
  if ( (getPowSign() != Padd.getPowSign()) && (Order()*Padd.Order() > 0) ) {
    printf( "Error: (Polynomial.cc) Addition of polynomials with opposite power sign...\n" );
    return *this;
  } 

  // Determine the polynomial with the highest order perform addition
  // operation accordingly
  if (Order() > Padd.Order()) {
    P1_ai = Head;
    P2_ai = Padd.Head;
    for (i = 0 ; i < Padd.size() ; i++) {
      *P1_ai = *P1_ai + *P2_ai;
      P1_ai++;
      P2_ai++;
    }
  }
  else {
    Pacc = Padd;
    P1_ai = Pacc.Head;
    P2_ai = Head;
    for (i = 0 ; i < size() ; i++) {
      *P1_ai = *P1_ai + *P2_ai;
      P1_ai++;
      P2_ai++;
    }
    *this = Pacc;
  }

  return *this;
  
}
/*----------------------------------------------------------------------*/


/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::operator-=( Pdiff ) : Computes the difference of "this"  
                                     polynomial and P2 and assigns it to 
                                     "this" polynomial.
          Pdiff   : (Polynomial&) The right side operant of the 
                    addition. Access via reference
                    
          return : (Polynomial&) Resulting polynomial, Pdiff = P - Psum,
                   Accessed via reference.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial& Polynomial::operator-=( const Polynomial& Pdiff ){

  Polynomial Pacc;
  int        i;
  double     *P1_ai;
  double     *P2_ai;

  // Error check: If non of the polynomials is a constant and the power
  // sign convension of these two polynomials do not match give an error
  // message and exit
  if ( (getPowSign() != Pdiff.getPowSign()) && (Order()*Pdiff.Order() > 0) ) {
    printf( "Error: (Polynomial.cc) Subtraction of polynomials with opposite power sign...\n" );
    return *this;
  } 

  // Determine the polynomial with the highest order perform addition
  // operation accordingly
  if (Order() > Pdiff.Order()) {
    P1_ai = Head;
    P2_ai = Pdiff.Head;
    for (i = 0 ; i < Pdiff.size() ; i++) {
      *P1_ai = *P1_ai - *P2_ai;
      P1_ai++;
      P2_ai++;
    }
  }
  else {
    Pacc = Pdiff;
    P1_ai = Pacc.Head;
    P2_ai = Head;
    for (i = 0 ; i < size() ; i++) {
      *P1_ai = *P1_ai - *P2_ai;
      P1_ai++;
      P2_ai++;
    }
    *this = Pacc;
  }

  return *this;

}
/*----------------------------------------------------------------------*/




/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::operator*=( P2 ) : Computes the multiplication of "this" 
                                  polynomial and P2 and assigns it to 
                                  "this" polynomial.

          Pmult  : (Polynomial&) The right side operant of the 
                   multiplication. Access via reference
                    
          return : (Polynomial&) Resulting polynomial, Psum = P * P2,
                   Accessed via reference.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial& Polynomial::operator*=( const Polynomial& Pmult ){

  Polynomial Pacc;
  int        i1, i2;
  
  // Check if both operants use the samesign convention
  if (( getPowSign() != Pmult.getPowSign() ) && ( Order() * Pmult.Order() > 0 )){
    printf( "Error: (Polynomial.cc) Multiplication of polynomials with opposite power sign...\n");
    return *this;
  }

  // Multiply the polynomials by multiplying and adding the coefficients
  Pacc.Redefine( Order() + Pmult.Order() + 1 );

  for (i1 = 0 ; i1 < size() ; i1++) {
    for (i2 = 0 ; i2 < Pmult.size() ; i2++) {
      Pacc.setEntry( i1+i2, 
                     Pacc.getEntry(i1+i2) + getEntry(i1) * Pmult.getEntry(i2) );        
    }
  }

  if ( Order() > 0 )
    Pacc.setPowSign( getPowSign() );
  else
    Pacc.setPowSign( Pmult.getPowSign() );

  *this = Pacc;

  return *this;

}
/*----------------------------------------------------------------------*/




/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::operator<<=( Psub ) : Substitutes the polynomial, Psub,
                                     instead of the independent variable
                                     of "this" polynomial generating a
                                     new polynomial, P(Psub(x)). Result is
                                     stored in "this".

          Psub   : (Polynomial&) The polynomial to be substituted into
                   "this" polynomial. Access via reference

          return : (Polynomial&) Resulting polynomial, 
                   Pres(x) = P(Psub(x)), Accessed via reference.
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
Polynomial& Polynomial::operator<<=( const Polynomial& Psub ){

  Polynomial Pacc(0);
  Polynomial Psub_pow_i(1);
  Polynomial Pinsert(0);
  double     *Pntr;
  int        i;

  // Check if this polynomial uses negative power sign convention which 
  // is not supported
  if (getPowSign() == -1) {
    printf( "Error: (Polynomial.cc) Substitution into a negative power polynomial...");
    return *this;
  }

  // Substitute the polynomial
  Pntr = Head;
  Pacc.setPowSign( Psub.getPowSign() );
  Psub_pow_i.setPowSign( Psub.getPowSign() );  

  for (i = 0 ; i <= Order() ; i++) {
    Pinsert     = Psub_pow_i;
    Pinsert    *= *Pntr;
    Pacc       += Pinsert;
    Psub_pow_i *= Psub;
    Pntr++;
  }

  *this = Pacc;

  return *this;
}
/*----------------------------------------------------------------------*/





/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* Polynomial::display( Pname ) : Prints the polynomial to the standard 
                                  output in a formated form standard output

++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void Polynomial::display( char* Pname ){

  int i;

  // Print the name and the constant of the poly
  printf( " %s = %f ", Pname, getEntry(0) );

  // Print the rest of the terms
  for (i = 1 ; i < size() ; i++) {
    if (PowSign == 1)
      printf("%+f x^%d ", getEntry(i), i);
    else
      printf("%+f x^{-%d} ", getEntry(i), i);
  }

  // Finalize by carriage return
  printf("\n");

} 
/*----------------------------------------------------------------------*/
