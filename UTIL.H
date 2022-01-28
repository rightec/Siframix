/**
@file		util.h
@brief		Utilities functions.

@author		
@date		18/01/2011
@version	01.0
*/

// Alcune utility per le operazioni sulle stringhe e sui files

// Costanti varie per la outerr


/* 
enum DBF_OPEN_ERR {                 // errori riportati ??
    NOERR, NOT_EXIST, NOT_RW, OPEN_ERR, NO_DBF_FILE, NO_MEM, READ_ERR,
    };

int  exist(char *name);
int  erase(char *name);
int  may_r_w(char *name);

int  fcopy(char *fdst, char *fsrc);
void OutErr(int  err, char *fname);

*/
#ifndef UTIL_H
#define UTIL_H

#include "global.h"


#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define roundUp(x) ((long)((x)+0.999)) 

float my_pow10(short value);
void strset(char *s, int filler);

void ltrim(char *s);
void rtrim(char *s);
byte *skipWhites(byte *buf);
byte *packWhites(byte *buf);
void cdate(char *s);
void cdaten(char *s);
int  sumstr(char *dst, char *src, int  len, int  val);
int  fsumstr(char *dst, char *src, int  maxlen, int  minlen, int  val);
void chgext(char *fname, char *ext);
int  addstr(char *dest, const char *source);
int  pperc(char *s, float base, float scost);
void IntToHex(int num, char* hex, unsigned int minNumChar);
int HexToInt(int& num, char* hex, int len);
int _1on8_to_int(byte m_bcd);
int windowFilter(short* Val, short *ValToFilter, int numChan, int CurrentIndex, int FilterDim);
void bubble_sort_array(int num_val, dword *array);
#endif

