#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

//#define DISABLE_DEBUG_PRINTMACRO
#ifndef DISABLE_DEBUG_PRINTMACRO
#include "../../Win32-simulator-MSVC/Supporting_Functions/supporting_functions.h"
#define DEBUG_PRINTMNUM(enable, s, u) do{\
                                     if(enable)\
                                       vPrintStringAndNumber(s, u);\
                                    }while(0)\

#define DEBUG_PRINTM(enable, s) do{\
                                    if(enable)\
                                        vPrintString(s);\
                                }while(0)\

#else 
#define DEBUG_PRINTMNUM(enable, s, u)
#define DEBUG_PRINTM(enable, s)
#endif // !DISABLE_DEBUG_PRINTMACRO

#endif // !DEBUG_PRINT_H