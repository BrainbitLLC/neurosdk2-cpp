#ifndef LIB_EXPORT_H
#define LIB_EXPORT_H
    #ifdef _WIN32
        #ifdef _MSC_VER
            #pragma warning(disable: 4251)
        #endif
        #define SDK_SHARED __declspec(dllexport)
    #elif defined(__GNUC__)
        //  GCC
        #define SDK_SHARED  __attribute__((visibility("default")))
    #else
        #define SDK_SHARED
    #endif
#endif // LIB_EXPORT_H
