/// @file utility.h
/// @brief Utility declaration / function prototype

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <stdio.h>
#include <string.h>

/*****************************************/
/* Debug mode - redirect printf function */
#ifdef _DEBUG_PRINTF_
// Debug support function
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int char)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* _DEBUG_PRINTF_ */
/****************************************/


#endif
