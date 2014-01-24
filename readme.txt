README FILE

roadtrack - program
functionality : tracking road network using region based descriptor

main.h          : parameters for PC version

REQUIRED LIBRARIES
1. OpenCV       : at least 2.3.1
2. Levmar       : levenberq marquardt optimization library 
                  http://users.ics.forth.gr/~lourakis/levmar/ 
3. FreeGLUT     
4. LAPACK & BLAS

Prepare the configuration of the libraries using vsprops for windows

FOLDERS and FILES

main.cpp        : the main program using GLUT
main.h          : parameters file
data            : folder for charset

Region.hpp      : get blobs/outline from one image
Tracker.hpp     : match and track methods

USAGE
roadtrac image name.txt annotations.txt