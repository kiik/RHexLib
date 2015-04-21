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
 * $Id: SymbolTable.cc,v 1.3 2001/08/12 18:56:10 ulucs Exp $
 *
 * This file implements the functions declared in SymbolTable.h
 *
 * Created       : Uluc Saranli, 12/17/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "ModuleManager.hh"
#include "SymbolTable.hh"

// Symbol::Symbol : Class constructor
Symbol::Symbol(void) {

  name[0] = 0;
  next = NULL;

  type = SYMBOL_INVALID;

  fvalue = new Floats;
  svalue = new Strings;
}

Symbol::~Symbol( ) {

  delete fvalue;
  delete svalue;
}


Symbol &Symbol::operator=( const Symbol &rhs ) {

  if( this == &rhs ) return *this; // prevent self-assignment

  delete fvalue;
  delete svalue;

  fvalue = new Floats( rhs.fvalue->getArray(), rhs.fvalue->getCount() );
  svalue = new Strings( rhs.svalue->getArray(), rhs.svalue->getCount() );

  return *this;
}

void Symbol::setFarray( float *f, int count ) {

  // Delete the old Floats array
  delete fvalue;

  // Create a new one with the new contents
  fvalue = new Floats( f, count );

  // Automatically set the type
  type = SYMBOL_FLOAT;
}

void Symbol::setSarray( char **s, int count ) {

  // Delete the old Strings array
  delete svalue;

  // Create the new one with the new contents
  svalue = new Strings( s, count );

  // Automatically set the type
  type = SYMBOL_STRING;
}

// SymbolTable::SymbolTable : Class constructor
SymbolTable::SymbolTable( void ) {

  numSymbols = 0;
  first = NULL;
  last = NULL;

}

// SymbolTable::SymbolTable : Class destructor
SymbolTable::~SymbolTable() {

  Symbol *curSymbol, *nextSymbol;

  for ( nextSymbol = curSymbol = first; nextSymbol != NULL; 
        curSymbol = nextSymbol) {

    nextSymbol = curSymbol->next;
    delete curSymbol;
  }

  first = NULL;
  last = NULL;

}

// SymbolTable::get : Searches for a named symbol
Symbol *SymbolTable::get( const char *name ) {

  Symbol *curSymbol;

  for ( curSymbol = first; curSymbol != NULL; curSymbol = curSymbol->next ) {

    if ( strcmp( name, curSymbol->name ) == 0) {
      return curSymbol;
    }
  }

  return NULL;
}

// SymbolTable::get : Inserts a symbol entry to the table
Symbol *SymbolTable::put( Symbol *sym ) {

  if ( last == NULL ) {

    first = last = sym;

  } else {

    last->next = sym;
    sym->next = NULL;
    last = sym;

  }

  numSymbols++;

  return sym;
}

// SymbolTable::putFloat : Creates and inserts a number symbol entry
// to the table
Symbol *SymbolTable::putFloat( const char *name, float val ) {

  Symbol *sym;

  if ( ( sym = new Symbol() ) == NULL )
    return NULL;

  sym->setName( name );
  sym->setFarray( &val, 1 );

  return put( sym );
}

// SymbolTable::putString : Creates and inserts a string symbol entry
// to the table
Symbol *SymbolTable::putString( const char *name, char *val ) {

  Symbol *sym;

  if ( ( sym = new Symbol() ) == NULL )
    return NULL;

  sym->setName( name );
  sym->setSarray( &val, 1 );

  return put( sym );
}

// SymbolTable::runScript : Reads in and adds symbol definitions from a file
void SymbolTable::runScript( ScriptType inputType, const char *file_or_str ) {

  int       errno;

  // Start the tokenizer
  if ( ( errno = tokenizer.open( inputType, file_or_str ) ) < 0 ) {
    MMWarning( "SymbolTable::runScript", "Error opening script file!" );
    return;
  }

  // Read and add all symbol definitions
  while ( parse_expression() );

  // Close the tokenizer
  tokenizer.close();

}

// SymbolTable::readFloat : Reads a float parameter. If not found, 
//   returns the supplied default value.
float SymbolTable::readFloat( const char *name, const float deflt ) {
  Symbol    *sym;

  if ( ( sym = get( name ) ) != NULL && sym->getType() == SYMBOL_FLOAT )
    return sym->getFvalue( 0 );
  else
    return deflt;

}

// SymbolTable::readArray : Reads a number array parameter. If not found, 
//   returns an empty floats struct.
Floats SymbolTable::readArray( const char *name ) {

  Symbol *sym;
  Floats deflt;

  if ( ( sym = get( name ) ) != NULL && sym->getType() == SYMBOL_FLOAT )
    return sym->getFarray();
  else
    return deflt;

}

// SymbolTable::readString : Reads a string parameter. If not found, 
//   returns the supplied default value.
void SymbolTable::readString( const char *name, char *str, 
                              const char *deflt ) {

  Symbol    *sym;

  if ( ( sym = get( name ) ) != NULL && sym->getType() == SYMBOL_STRING ) {
    strcpy( str, sym->getSvalue( 0 ) );
    return;

  } else {

    strcpy( str, deflt );
    return;

  }
}

// SymbolTable::readArray : Reads a string array parameter. If not found, 
//   returns an empty strings struct.
Strings SymbolTable::readStrArray( const char *name ) {

  Symbol *sym;
  Strings deflt;

  if ( ( sym = get( name ) ) != NULL && sym->getType() == SYMBOL_STRING )
    return sym->getSarray();
  else
    return deflt;

}

// SymbolTable::printSymbols : 
//
//  Prints the current symbols and their values to stdout.
// for debugging purposes only.

void SymbolTable::printSymbols( void ) {

  Symbol * sym = first;
  Floats   fval;
  Strings  sval;
  int      i;

  while ( sym ) {
    
    if ( sym->getType() == SYMBOL_FLOAT ) {

      fval = sym->getFarray();
      printf( "\nName  : %s\n", sym->getName() );
      printf( "Value : " );

      if ( fval.getCount() != 1 )
        printf( "{" );
      for ( i = 0; i < fval.getCount(); i++ )
        printf( "%e ", fval.get(i) );
      if ( fval.getCount() != 1 )
        printf( "}\n" );
      else
        printf( "\n" );

    } else if ( sym->getType() == SYMBOL_STRING ) {

      sval = sym->getSarray();
      printf( "\nName  : %s\n", sym->getName() );
      printf( "Value : " );

      if ( sval.getCount() != 1 )
        printf( "{" );
      for ( i = 0; i < sval.getCount(); i++ )
        printf( "\"%s\" ", sval.get(i) );
      if ( sval.getCount() != 1 )
        printf( "}\n" );
      else
        printf( "\n" );
    }

    sym = sym->next;
  }
}

bool SymbolTable::parse_value( Symbol *sym ) {

  Token token;
  float fvalue;
  float farray[MAX_ARRAY_LENGTH];
  char  *svalue;
  char  *sarray[MAX_ARRAY_LENGTH];
  int count;

  // The possible values for symbols are:
  //
  // String : TOKEN_STRING
  //
  // Number : TOKEN_FLOAT | TOKEN_INTEGER
  //
  // Float Array : "{" ( ( TOKEN_FLOAT | TOKEN_INTEGER ) "," )... "}"
  //
  // String Array : "{" ( ( TOKEN_STRING | TOKEN_STRING ) "," )... "}"
  //
  GET_TOKEN( tokenizer, &token );

  if ( token.type == TOKEN_FLOAT ) {
    // Acquire a floating point value for the symbol
    fvalue = token.fvalue;
    sym->setFarray( &fvalue, 1 );

  } else if ( token.type == TOKEN_INTEGER ) {
    // Acquire an integer value for the symbol

    fvalue = token.ivalue;
    sym->setFarray( &fvalue, 1 );

  } else if ( token.type == TOKEN_STRING ) {
    // Acquire a string value for the symbol

    svalue = token.svalue;
    sym->setSarray( &svalue, 1 );

  } else if ( token.type == TOKEN_LBRACKET ) {
    // The value is an array of numbers.
    count = 0;

    GET_TOKEN( tokenizer, &token );

    if ( token.type == TOKEN_FLOAT || token.type == TOKEN_INTEGER ) {
      // This must be a float array. Proceed accordingly

      // Get all the numbers in the array
      while ( token.type != TOKEN_RBRACKET ) {

        if ( token.type == TOKEN_FLOAT ) {
          // Acquire a floating point value for the symbol
          farray[count++] = token.fvalue;

        } else if ( token.type == TOKEN_INTEGER ) {
          // Acquire an integer value for the symbol
          farray[count++] = token.ivalue;

        } else {
          // Oops, unexpected token here
          MMWarning("SymbolTable::parse_value", "Invalid array element!" );
          return false;
        }
        GET_TOKEN( tokenizer, &token );
      }
      // Set the symbol value to the array
      sym->setFarray( farray, count );

    } else if ( token.type == TOKEN_STRING ) {
      // This is a string array. Proceed accordingly.

      // Get all the strings in the array
      while ( token.type != TOKEN_RBRACKET ) {

        if ( token.type == TOKEN_STRING ) {
          // Acquire string value for the symbol
          sarray[count] = new char[ strlen( token.svalue ) + 1 ];
          strcpy( sarray[ count ], token.svalue );
          count++;

        } else {
          // Oops, unexpected token here
          MMWarning("SymbolTable::parse_value", "Invalid array element!" );
          return false;
        }
        GET_TOKEN( tokenizer, &token );
      }

      // Set the symbol value to the array
      sym->setSarray( sarray, count );

    } else {

      // Unexpected token type.
      MMWarning("SymbolTable::parse_value", "Invalid array element!" );
      return false;

    }


  } else {
    // Oops, unexpected token here
    MMWarning("SymbolTable::parse_value", "Invalid value assignment!" );

    return false;
  }

  return true;
}

bool SymbolTable::parse_expression( void ) {

  Symbol *sym, *oldSymbol;
  Token  token;

  // First look for the symbol name
  GET_TOKEN( tokenizer, &token );
  
  if ( token.type == TOKEN_EOF )
    return false;

  if ( token.type != TOKEN_SYMBOL ) {
    MMWarning( "SymbolTable::parse_expression", 
               "Symbol token expected. Aborting!" );
    return false;
  }

  // If the symbol already exists, retrieve it. Otherwise, create a new one
  if ( ( oldSymbol = get( token.svalue ) ) != NULL ) {

    sym = oldSymbol;

  } else {

    sym = new Symbol;
  }

  // Set the name of the symbol
  sym->setName( token.svalue );

  // Expect an equal sign
  EXPECT_TOKEN( tokenizer, TOKEN_EQUAL, &token );

  // Parse the value and fill in the symbol entry
  if ( ! parse_value( sym ) ) {
    if ( sym != oldSymbol )
      delete sym;

    return false;
  }

  // Insert the table entry
  if ( sym != oldSymbol )
    put( sym );

  // Expect a semicolon at the end
  EXPECT_TOKEN( tokenizer, TOKEN_SEMICOLON, &token );

  return true;
}

