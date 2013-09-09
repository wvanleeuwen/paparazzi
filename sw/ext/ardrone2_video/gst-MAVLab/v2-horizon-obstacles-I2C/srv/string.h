#include "stdlib.h"

extern int  strcmp(char *, char *);
extern int strncmp(char *, char *, int);
extern char *strchr(char *, char);
extern void strcpy(char *, char *);
extern char *strncpy(char *, char *, int);
extern char *strdup(char *);
extern int strlen(char *);
extern int isdigit(char);
extern int  atoi(char *);
extern void itoa(int, char *);
extern void reverse(char *);
extern void memcpy(unsigned char *, unsigned char *, int);
extern void memcpy2(unsigned char *, unsigned char *, int);
extern void *memmove (void *dst, void *src, int count);
extern void memset(unsigned char*, unsigned char, int);
extern unsigned int ctoi(unsigned char);
#define xmemset memset
#define xmemcpy memcpy
extern unsigned int atoi_b16(char *s);
extern unsigned int atoi_b10(char *s);
extern float atof(char *s);
extern char *strpbrk(const char *str, const char *set);
extern char *strtok(char *s, const char *delim);
extern char *strstr (const char * str1,const char * str2);



//
// strtoksafe: thread-safe version of strtok.
//
// Same arguments and return value as strtok, with addition argument, last, which keeps the context
// for repeated calls to strtoksafe
//
extern char *strtoksafe(char *s, const char *delim, char **last);


//
// strnstr: search a buffer of a given size for a string
//
// Similar to strstr, except the string to search is instead a buffer with a count, and embedded zeroes
// in the buffer do not terminate the buffer
//
extern char * strnstr (         // Returns pointer to found string in str1 or NULL
const char * str1,              // Buffer to search. Null characters do NOT terminate the buffer (i.e., it may
                                //  contain embedded null chars that are considered characters)
const char * str2,              // Pattern to search for, null-terminated string
int n);                         // Size of str1 buffer, in characters


//
// strReplace: replace a pattern in a string with another string (i.e., delete the pattern and insert the new string)
//
extern BOOL strReplace (        // Returns TRUE if pattern found and replaced successfully
char *      dest,               // Destination string
int         destMax,            // Maximum size of destination string buffer, to accomodate replacement
char *      pattern,            // Pattern to search for
char *      replacement);       // Replacement string



//
// getUnaligned32: get a 32-bit value from an unaligned address
//
extern unsigned getUnaligned32 (    // Returns value
void *          p);                 // Pointer to 32-bit value, need not be 32-bit aligned



