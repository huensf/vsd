/*
 * author: Nicolas Van der Noot, Olivier Lantsoght
 * date: September 26 2017
 * 
 * Get absolute paths to the source project root (first CMakeLists.txt),
 * to the binaries and to Robotran installation folder.
 *
 * A header file called 'cmake_config.h' is then automatically generated in the build directory
 */
#define PROJECT_SOURCE_DIR "C:/Users/Lucas/Documents/Robotran_Projects/vsd/workR"
#define PROJECT_BINARY_DIR "C:/Users/Lucas/Documents/Robotran_Projects/vsd/workR/build"
#define BUILD_PATH_REL "build"
#ifdef UNIX
    #define BUILD_PATH  "C:/Users/Lucas/Documents/Robotran_Projects/vsd/workR/build"
#else
    #ifdef _DEBUG
        #define BUILD_PATH  "C:/Users/Lucas/Documents/Robotran_Projects/vsd/workR/build/Debug"
    #else
        #define BUILD_PATH  "C:/Users/Lucas/Documents/Robotran_Projects/vsd/workR/build/Release"
    #endif
#endif
