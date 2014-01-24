README FILE

roadtrac - program
functionality : AR maps using road network map tracking

main.h          : parameters 

REQUIRED LIBRARIES
1. OpenCV       : at least 2.3.1
2. Levmar       : levenberq marquardt optimization library 
                  http://users.ics.forth.gr/~lourakis/levmar/ 
3. FreeGLUT     
4. LAPACK & BLAS

FOLDERS and FILES

main.cpp        : the main program using GLUT
main.h          : parameters file
data            : folder for charset
Region.hpp      : get blobs/outline from one image
Tracker.hpp     : match and track method

USAGE
roadtrac image name.txt annotations.txt
