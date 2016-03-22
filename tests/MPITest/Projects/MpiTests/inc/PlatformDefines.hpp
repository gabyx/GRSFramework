#ifndef PLATFORM_DEFINES
#define PLATFORM_DEFINES



#if defined _WIN32 || defined _WIN64
    #define HOLD_SYSTEM { system("pause"); }
#elif defined __unix__  || defined __unix || defined __apple__
    #define HOLD_SYSTEM { \
    printf("Press 'Enter' to exit the program ..."); \
    while (getchar() != '\n'); \
    printf("\n\n");}
#endif


#if defined _WIN32 || defined _WIN64
    #include <windows.h>
    #define SLEEP(_ms_)  Sleep(_ms_);
#elif defined __unix__  || defined __unix || defined __apple__
    #include <unistd.h>
    #define SLEEP(_ms_)  sleep(_ms_);
#endif


#endif
