/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  string.c - string library for the SRV-1 Blackfin robot.
 *    modified from main.c - main control loop for SRV-1 robot
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "malloc.h"
#include "stdlib.h"

int strcmp(char *s1, char *s2)
{
  while (*s1 == *s2++)
    if (*s1++ == 0)
        return (0);
  return (*s1 - *--s2);
}

int strncmp(char *s1, char *s2, int n)
{
  unsigned char u1, u2;
  while (n-- > 0)
    {
      u1 = (unsigned char) *s1++;
      u2 = (unsigned char) *s2++;
      if (u1 != u2)
    return u1 - u2;
      if (u1 == '\0')
    return 0;
  }
  return 0;
}

char *strchr(char *s, char c)
{
        for (;;)
        {
             if (*s == c) 
                    return s;
             if (!*s++) 
                    return 0;
        }
}

void strcpy(char *pDst, char *pSrc)
{
        while ((*pDst++ = *pSrc++) != '\0')
                continue;
}

char *strncpy(char *dst, const char *src, int n)
{
  if (n != 0) {
    char *d = dst;
    const char *s = src;
    do {
      if ((*d++ = *s++) == 0) {
        /* NUL pad the remaining n-1 bytes */
        while (--n != 0)
          *d++ = 0;
        break;
      }
    } while (--n != 0);
  }
  return (dst);
}

int atoi(char *p)
{
    int n, f;

    n = 0;
    f = 0;
    for(;;p++) {
        switch(*p) {
        case ' ':
        case '\t':
            continue;
        case '-':
            f++;
        case '+':
            p++;
        }
        break;
    }
    while(*p >= '0' && *p <= '9')
        n = n*10 + *p++ - '0';
    return(f? -n: n);
}

int strlen(char *pStr)
{
    char *pEnd;

    for (pEnd = pStr; *pEnd != 0; pEnd++)
        continue;

    return pEnd - pStr;
}

char *strdup(char *s)
{
    char *result = (char*)malloc(strlen(s) + 1);
    if (result == (char*)0)
        return (char*)0;
    strcpy(result, s);
        return result;
}

#define SWAP_CHAR( x, y ) {char c; c = x; x = y; y = c;}

void reverse(char *t)
{
  int i,j;
  for(i = 0, j = strlen(t)-1; i < j; i++, j--)
    SWAP_CHAR(t[i], t[j]);
}

void itoa( int n, char *s )
{
    int i, sign;
    if ( ( sign = n ) < 0 )
        n = -n;
    i = 0;
    do {
        s[ i++ ] = (unsigned int)n % 10 + '0';
    } while (( n /= 10 ) > 0 );
    if ( sign < 0 )
        s[ i++ ] = '-';
    s[ i ] = '\0';
    reverse( s );
}

int isdigit(c)
 char c;
{
  if ((c>='0') && (c<='9'))
    return 1;
  else
    return 0;  
}

void memcpy (char *dst, char *src, int count)
{
   while (count--)
       *dst++ = *src++;
}

void memcpy2 (char *dst, char *src, int count)
{
    short *isrc, *idst;
    int ix;

    idst = (short *)dst;
    isrc = (short *)src;
    for (ix = (count/2) - 1; ix>=0; ix--)
        *idst++ = *isrc++;
}


#if 0
//
// This version sometimes only copied two bytes for reasons unknown
//
void memcpy (char *dst, char *src, int count)
{
   short *isrc, *idst;

   /* if count and pointers are even, transfer 16-bits at a time */
   if ((count & 0x00000001) || 
       ((int)dst & 0x00000001) || 
       ((int)src & 0x00000001)) {
       while (count--)
           *dst++ = *src++;
   } else {

       idst = (short *)dst;
       isrc = (short *)src;
       count /= 2;
       while (count--)
           *idst++ = *isrc++;
   }
}
#endif


void memset (char *dst, char ch, int count)
{
    while (count--)
        *dst++ = ch;
}

void *memmove (void *dst, void *src, int count)
{
    void *ret = dst;

    if (dst <= src || (char *)dst >= ((char *)src + count)) {
        while (count--) {
            *(char *)dst = *(char *)src;
            dst = (char *)dst + 1;
            src = (char *)src + 1;
        }
    }
    else {
        dst = (char *)dst + count - 1;
        src = (char *)src + count - 1;

        while (count--) {
            *(char *)dst = *(char *)src;
            dst = (char *)dst - 1;
            src = (char *)src - 1;
        }
    }

    return(ret);
}

unsigned int ctoi(unsigned char c) {
    if (c > '9')
        return (unsigned int)(c & 0x0F) + 9;
    else
        return (unsigned int)(c & 0x0F);
}

unsigned int atoi_b16(char *s) {
    // Convert two ascii hex characters to an int
    unsigned int result = 0;
    int i;
    for(i = 0; i < 2; i++,s++)  {
        if (*s >= 'a')
            result = 16 * result + (*s) - 'a' + 10;
        else if (*s >= 'A')
            result = 16 * result + (*s) - 'A' + 10;
        else
            result = 16 * result + (*s) - '0';
    }
    return result;
}

unsigned int atoi_b10(char *s) {
    // Convert two ascii decimal characters to an int
    unsigned int result = 0;
    int i;
    for(i = 0; i < 2; i++,s++)
        result = 10 * result + (*s) - '0';
    return result;
}

float atof(char *s)
{
    float a = 0.0;
    int e = 0;
    int c;
    while ((c = *s++) != '\0' && isdigit(c)) {
        a = a*10.0 + (c - '0');
    }
    if (c == '.') {
        while ((c = *s++) != '\0' && isdigit(c)) {
            a = a*10.0 + (c - '0');
            e = e-1;
        }
    }
    if (c == 'e' || c == 'E') {
        int sign = 1;
        int i = 0;
        c = *s++;
        if (c == '+')
            c = *s++;
        else if (c == '-') {
            c = *s++;
            sign = -1;
        }
        while (isdigit(c)) {
            i = i*10 + (c - '0');
            c = *s++;
        }
        e += i*sign;
    }
    while (e > 0) {
        a *= 10.0;
        e--;
    }
    while (e < 0) {
        a *= 0.1;
        e++;
    }
    return a;
}

char *strpbrk(const char *str, const char *set) {
    while (*str != '\0')
        if (strchr(set, *str) == 0)
            ++str;
        else
            return (char *) str;

    return 0;
}

char *strtok(char *s, const char *delim)
{
    char *spanp;
    int c, sc;
    char *tok;
    static char *last;


    if (s == 0 && (s = last) == 0)
        return (0);

    /*
     * Skip (span) leading delimiters (s += strspn(s, delim), sort of).
     */
cont:
    c = *s++;
    for (spanp = (char *)delim; (sc = *spanp++) != 0;) {
        if (c == sc)
            goto cont;
    }

    if (c == 0) {        /* no non-delimiter characters */
        last = 0;
        return (0);
    }
    tok = s - 1;

    /*
     * Scan token (scan for delimiters: s += strcspn(s, delim), sort of).
     * Note that delim must have one NUL; we stop if we see that, too.
     */
    for (;;) {
        c = *s++;
        spanp = (char *)delim;
        do {
            if ((sc = *spanp++) == c) {
                if (c == 0)
                    s = 0;
                else
                    s[-1] = 0;
                last = s;
                return (tok);
            }
        } while (sc != 0);
    }
    /* NOTREACHED */
}



char *strtoksafe(char *s, const char *delim, char **last)
{
    char *spanp;
    int c, sc;
    char *tok;

    if (s == 0 && (s = *last) == 0)
        return (0);

    /*
     * Skip (span) leading delimiters (s += strspn(s, delim), sort of).
     */
cont:
    c = *s++;
    for (spanp = (char *)delim; (sc = *spanp++) != 0;) {
        if (c == sc)
            goto cont;
    }

    if (c == 0) {        /* no non-delimiter characters */
        *last = 0;
        return (0);
    }
    tok = s - 1;

    /*
     * Scan token (scan for delimiters: s += strcspn(s, delim), sort of).
     * Note that delim must have one NUL; we stop if we see that, too.
     */
    for (;;) {
        c = *s++;
        spanp = (char *)delim;
        do {
            if ((sc = *spanp++) == c) {
                if (c == 0)
                    s = 0;
                else
                    s[-1] = 0;
                *last = s;
                return (tok);
            }
        } while (sc != 0);
    }
    /* NOTREACHED */
}


char * strstr (
        const char * str1,
        const char * str2
        )
{
        char *cp = (char *) str1;
        char *s1, *s2;

        if ( !*str2 )
            return(char *)str1;

        while (*cp)
        {
                s1 = cp;
                s2 = (char *) str2;

                while ( *s1 && *s2 && !(*s1-*s2) )
                        s1++, s2++;

                if (!*s2)
                        return cp;

                cp++;
        }

        return 0;

}


char * strnstr (
        const char * str1,
        const char * str2,
        int n
        )
{
        char *cp = (char *) str1;
        char *s1, *s2;

        if ( !*str2 )
            return(char *)str1;

        while (n > 0)
        {
                s1 = cp;
                s2 = (char *) str2;

                int n2 = n;
                while ( n2 > 0  && *s2 && !(*s1-*s2) )
                        s1++, s2++, --n2;

                if (!*s2)
                        return cp;

                cp++;
                --n;
        }

        return 0;

}




BOOL        strReplace ( 
char *      dest,
int         destMax,     
char *      pattern,     
char *      replacement)
{
    char * targ = strstr (dest, pattern);
    if (targ != NULL) {
        int destLen = strlen (dest);
        int patLen = strlen (pattern);
        int repLen = strlen (replacement);
        if (destLen + repLen - patLen + 1 > destMax)
            return FALSE;
        memmove (targ + repLen, targ + patLen, (destLen - (targ - dest) - patLen + 1) * sizeof (char));
        memcpy (targ, replacement, repLen);
        return TRUE;
    }        
    else
        return FALSE;
}





unsigned    getUnaligned32 (
void *      p)
{
    unsigned char * c = p;
#ifdef _BIG_ENDIAN
    return c[3] + (c[2] << 8) + (c[1] << 16) + (c[0] << 24);
#else
    return c[0] + (c[1] << 8) + (c[2] << 16) + (c[3] << 24);
#endif
}



