#define STR_DIFF		2			//maximum difference for comparison
#define MINMATCH		10 			//minimum value for matching
#define IMG_SCALE		2.0			//the image scale for processing image
#define IMG_WIDTH		640			//image width
#define IMG_HEIGHT		480			//image height
#define MIN_DCE			5			//minimum value for relevance measure (3.5 = relmeas /l1+l2    )
#define FRAMESIZE		1024		//open gl frame size
#define DISTTOPOLY		20			//maximum distant to the polygon of reference
#define CLOSEPOINTS		0.5			//the minimum number of points

#define CHECKCONTOURSNUM 0			//check the number of contours in every frame
#define LIMITREGION		1			//constraining region, this is for non map objects
#define EXTERNAL		1			//for region extraction using external contour
#define MINPOINTS		12			//minimum points
#define MINAREA		   	100
#define UPDATE			1			//update the database
#define SAVE			0			//saving the screenshot 
#define CAST			1			//cast the float to upper integer
#define MINOVERLAP		20			//minimum overlap
#define LIGHT			1			//using opengl lighting
#define DRAWBORDER		0			//draw planar border
#define CAMERAID		1			//0 = default web cam, 1 = usb web cam
#define LMOPTIMIZE		1			//optimizing using levenberg marquardt optimization from lourakis (blas, lapack, levmar library are required)
#define LMOPTIMIZEINV	1			//fitting from the extracted contour to template, orelse fitting from the template to extracted contour
#define LMRAWCONTOUR	1			//using non simplified contour for optimization
#define LIMITPOLY		1			//limiting the number of polygon to the number of elements
#define LMRAWCONTOURN	80			//limit the contour only using n element
#define CLEANRAW		1			//cleaning the raw contours
#define LMMAXERR		0.0001		//maximum error
#define LMITER			100			//lm optimization iteration
#define SINGLE			0			//load only the first model in the database, good for tracking demo
#define LMOPTIMIZENEIGH 1			//optimizing the homography using the previous homography
#define MAXNEIGHDIST    10			//maximum average distance of neighboring frames
#define LOADLABELS		1			//load labels for the region
#define RECALLHIST		1			//using the history of detected ids. if the detection fail, match from the history of detected ids
#define MAT_TYPE CV_64FC1

#include "opencv.hpp"
#include "opencv2/legacy/legacy.hpp"

#include "Tracker.hpp"
Tracker m_tracker;

#include "Region.hpp"
Region reg;

#include <string>

//open GL header
#include <GL/glut.h>
#include <GL/gl.h>

//OpenGL text
#define DATA_CHARSET "data/charset"					//OpenGL text data
#include "text3d/text3d.h"
T3DText m_text; //! text

CvCapture *cap;
IplImage *im = 0; 
IplImage *bg = 0;
int wid;
GLuint *texture;
double projection[16];
double view[16];

bool m_save = false;								//variable to save region into database
bool m_rotated = true; //rotated labels

void resize(int w, int h);
void drawcg();
void display();
void draw(int id, IplImage* src);
void idle();
void setview(CvMat*r, CvMat*t, GLdouble *dst);
void process();
void setviewfromhomography(CvMat* homo);
void setprojection();
void setlight();
void save(IplImage *templimg, char* name);
void reset();
void close();
void keyboard(unsigned char key, int x, int y);
void readwindow();