//
// Debug.h
//

#ifdef _DEBUG

#define DebugStr(s)     uart1SendString ((unsigned char *) s)
#define DebugChar(c)    uart1SendChar ((unsigned char) c)

#else

#define DebugStr(s)
#define DebugChar(c)

#endif

