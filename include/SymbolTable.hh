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
 * $Id: SymbolTable.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * This file contains all the datatypes and function prototypes for
 * the SymbolTable class. The functions declared in this file are
 * defined in SymbolTable.c The symbol table facility provides a
 * repository for named symbols with string or real number values.
 *
 * Created       : Uluc Saranli, 12/17/1998
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _SYMBOLTABLE_HH
#define _SYMBOLTABLE_HH

// System includes
#include <string.h>

// Local includes
#include "types.hh"
#include "Tokenizer.hh"

// Constants -------------------------------------------------------
#define MAX_SYMBOL_NAMELEN  128
#define MAX_ARRAY_LENGTH    128

// Data types ------------------------------------------------------
typedef enum { SYMBOL_INVALID, SYMBOL_FLOAT, SYMBOL_STRING } SymbolType;
typedef enum { 
  SCRIPT_FILE = TOKEN_INPUT_FILE, 
  SCRIPT_STRING = TOKEN_INPUT_STRING 
} ScriptType;   

// Symbol class -------------------------------------------------
class Symbol {
private:

  Symbol     *next;
  char       name[MAX_SYMBOL_NAMELEN];
  SymbolType type;
  
  Floats     *fvalue;
  Strings    *svalue;

  friend class SymbolTable;


public:
  Symbol( void );
  ~Symbol( );

  Symbol &operator=( const Symbol &rhs );

  void        setName( const char *n ) { strcpy( name, n); };
  const char *getName( void ) { return name; };
  SymbolType  getType( void ) { return type; };

  void        setFarray( float *f, int count );
  void        setSarray( char **s, int count );

  Floats      getFarray( void ) { return *fvalue; };
  float       getFvalue( unsigned int index ) { return fvalue->get( index ); };
  Strings     getSarray( void ) { return *svalue; };
  const char *getSvalue( unsigned int index ) { return svalue->get( index ); };

};

// SymbolTable class --------------------------------------------
class SymbolTable {
  
public:

  SymbolTable( void );
  ~SymbolTable();
  
  Symbol *get( const char *name );
  Symbol *put( Symbol *sym );
  Symbol *putFloat( const char *name, float val );
  Symbol *putString( const char *name, char *val );

  void    runScript( ScriptType inputType, const char *file_or_str );
  void    loadFile( const char *filename ) { runScript( SCRIPT_FILE, filename ); };
  float   readFloat( const char *name, const float deflt );
  Floats  readArray( const char *name );
  void    readString( const char *name, char *str, const char *deflt );
  Strings readStrArray( const char *name );

  void    printSymbols( void );

private:

  int numSymbols;
  
  Symbol *first;
  Symbol *last;

  // Tokenizer object for parsing input files
  Tokenizer tokenizer;

  bool parse_value( Symbol *sym );
  bool parse_expression( void );
};


#endif
