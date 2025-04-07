#pragma once
#include <execinfo.h>
#include <cxxabi.h> // For demangling
#include <stdio.h>
#include <stdlib.h>
#include <string>

static std::string demangle(const char* name) {
    int status;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0) ? demangled : name;
    free(demangled);
    return result;
}

static void stacktrace(const char * file, int line) {
    void* callstack[128];
    int frames = backtrace(callstack, 128);
    char** strs = backtrace_symbols(callstack, frames);
    printf("\033[0;31m%s:%u\033[0m\n", file, line);
    for (int i = 1; i < frames; ++i) {
        // Extract the mangled name between parentheses
        char* mangled_name = nullptr;
        char* offset_begin = nullptr;
        char* offset_end = nullptr;
        
        // Find parentheses
        for (char* p = strs[i]; *p; ++p) {
            if (*p == '(') {
                mangled_name = p + 1;
            } else if (*p == '+') {
                offset_begin = p;
            } else if (*p == ')') {
                offset_end = p;
                break;
            }
        }
        
        if (mangled_name && offset_begin && offset_end && 
            mangled_name < offset_begin) {
            *offset_begin = '\0';
            std::string demangled = demangle(mangled_name);
            printf("\033[0;31m%s+%s\033[0m\n", demangled.c_str(), offset_begin + 1);
        } else {
            printf("\033[0;31m%s\033[0m\n", strs[i]);
        }
    }
    
    free(strs);
}

#define print_stacktrace()                                                                          \
	(void)(stacktrace(__FILE__, __LINE__))

// void print_stacktrace() {
//     void* callstack[128];
//     int frames = backtrace(callstack, 128);
//     char** strs = backtrace_symbols(callstack, frames);
    
//     for (int i = 0; i < frames; ++i) {
//         printf("%s\n", strs[i]);
//     }
    
//     free(strs);
// }