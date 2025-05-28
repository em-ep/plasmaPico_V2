// debug.h
#pragma once

// Debug levels (0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG)
#define DEBUG_LEVEL 3 // Leave at default value. Overwrite per-file or via compiler flags

// Helper macros
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Level 1: Critical errors (always enabled if DEBUG_LEVEL >= 1)
#if DEBUG_LEVEL >= 1
    #define LOG_ERROR(fmt, ...) \
        printf("[ERROR] %s(): " fmt "\n", __func__, ##__VA_ARGS__)
#else
    #define LOG_ERROR(fmt, ...)
#endif

// Level 2: Warnings (unexpected states)
#if DEBUG_LEVEL >= 2
    #define LOG_WARN(fmt, ...) \
        printf("[WARN]  %s(): " fmt "\n", __func__, ##__VA_ARGS__)
#else
    #define LOG_WARN(fmt, ...)
#endif

// Level 3: General info (function flow, key states)
#if DEBUG_LEVEL >= 3
    #define LOG_INFO(fmt, ...) \
        printf("[INFO]  %s(): " fmt "\n", __func__, ##__VA_ARGS__)
#else
    #define LOG_INFO(fmt, ...)
#endif

// Level 4: Verbose debugging (variable dumps, etc.)
#if DEBUG_LEVEL >= 4
    #define LOG_DEBUG(fmt, ...) \
        printf("[DEBUG] %s(): " fmt "\n", __func__, ##__VA_ARGS__)
#else
    #define LOG_DEBUG(fmt, ...)
#endif