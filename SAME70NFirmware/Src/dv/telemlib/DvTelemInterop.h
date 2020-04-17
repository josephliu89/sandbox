#ifndef DV_TELEM_INTEROP_H
#define DV_TELEM_INTEROP_H

#if defined(DV_OS_WINDOWS) || defined(DV_OS_LINUX)

    // inherit values from the pre-compiled header values

#else

    #include <assert.h>
    #include <stdbool.h>
    #include <stddef.h>
    #include <stdint.h>
    #include <string.h>

    #define nullptr NULL
    #define DV_staticAssert(x)
    #define DV_unused(PP_name) ((void)(PP_name))
    #define DV_ensure(x) assert(x)
    #define DV_ensureAlignment(ptr, alignment) assert((((size_t)ptr) % alignment) == 0)

    #ifndef DV_LOGGING_FUNCTIONS
        #define HIDE_DEBUG_SPEW 1

        #define DV_LOGGING_FUNCTIONS
        #define DV_EOL "\r\n"
        #define DV_log(loglevel, fmt, ...) printf((loglevel fmt DV_EOL), ##__VA_ARGS__)
        #define DV_info(fmt, ...) DV_log("[info]\t", fmt, ##__VA_ARGS__)
        #define DV_warn(fmt, ...) DV_log("[warn]\t", fmt, ##__VA_ARGS__)
        #define DV_error(fmt, ...) DV_log("[error]\t", fmt, ##__VA_ARGS__)

        #if HIDE_DEBUG_SPEW
            #define DV_debug(fmt, ...)
        #else
            #define DV_debug(fmt, ...) DV_log("[debug]\t", fmt, ##__VA_ARGS__)
        #endif
    #endif

    #ifdef _MSC_VER

        #include "codeanalysis/warnings.h"

        #define DV_alignOf(_TYPE_) __alignof(_TYPE_)

        #define DV_warnPush() __pragma(warning(push)) __pragma(warning(disable : ALL_CODE_ANALYSIS_WARNINGS))
        #define DV_warnPushLevel(PP_level) __pragma(warning(push, PP_level)) __pragma(warning(disable : ALL_CODE_ANALYSIS_WARNINGS))
        #define DV_warnPop() __pragma(warning(pop))
        #define DV_warnDisableMSVC(PP_warningNumber) __pragma(warning(disable: PP_warningNumber))
        #define DV_warnDisableGCC(PP_warningName)

        DV_warnDisableMSVC(4100) // unreferenced formal parameter

    #elif defined(__ATPL360B__)

        #define DV_alignOf(_TYPE_) __alignof__(_TYPE_)

        #define DV_warnPush()
        #define DV_warnPushLevel(PP_level)
        #define DV_warnPop()
        #define DV_warnDisableMSVC(PP_warningNumber)
	    #define DV_warnDisableGCC_detail(PP_warningName) #PP_warningName
	    #define DV_warnDisableGCC(PP_warningName) _Pragma(DV_warnDisableGCC_detail(GCC diagnostic ignored PP_warningName))

    #elif defined(__clang__) || defined(__GNUC__)

        #define DV_alignOf(_TYPE_) __alignof__(_TYPE_)

        // Note that Clang works fine with GCC's macros.   
	    #define DV_warnPush() _Pragma("GCC diagnostic push")                                                                                
	    #define DV_warnPushLevel(PP_level) DV_warnPush()
	    #define DV_warnPop() _Pragma("GCC diagnostic pop")
	    #define DV_warnDisableMSVC(PP_warningName)	
	    #define DV_warnDisableGCC_detail(PP_warningName) #PP_warningName
	    #define DV_warnDisableGCC(PP_warningName) _Pragma(DV_warnDisableGCC_detail(GCC diagnostic ignored PP_warningName))

    #else
        #pragma error("Compiling on unknown platform. Good luck!")
    #endif

#endif // DV_OS_WINDOWS

#endif // DV_TELEM_INTEROP_H 
