#include "main.h"

void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle()
{
	glutPostRedisplay();
}

void reset()
{
	reg.Clear();
	m_tracker.Clear();
}

void keyboard(unsigned char key, int x, int y) {

	switch (key) {
	case 0x1b:
		reset();
		close();
		break;
	case 's':
		m_save = !m_save;
		break;
	case 'c':
		reset();
		break;
	case 'h':
		break;
	case 'r': //rotated lables
		m_rotated = !m_rotated;
		break;
	default:
		break;
	}
}

void draw(int id, IplImage* src)
{
	if (!src)
		return;

	glutSetWindow(id);

	double wr = static_cast<double>(src->width) / static_cast<double>(FRAMESIZE);
	double hr = static_cast<double>(src->height)/  static_cast<double>(FRAMESIZE);

	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glPixelStorei(GL_UNPACK_ALIGNMENT, src->align);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src->width, src->height, GL_BGR_EXT, GL_UNSIGNED_BYTE, src->imageData);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);
	glTexCoord2d(wr, 0.0);		glVertex2d(1, 1);
	glTexCoord2d(wr, hr);		glVertex2d(1, -1);
	glTexCoord2d(0.0, hr);		glVertex2d(-1, -1);
	glTexCoord2d(0.0, 0.0);		glVertex2d(-1, 1);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void setview(CvMat*r, CvMat*t, GLdouble *dst) {
	dst[0] = cvmGet(r, 0, 0);
	dst[1] = cvmGet(r, 1, 0);
	dst[2] = cvmGet(r, 2, 0);
	dst[3] = 0.0;

	dst[4] = cvmGet(r, 0, 1);
	dst[5] = cvmGet(r, 1, 1);
	dst[6] = cvmGet(r, 2, 1);
	dst[7] = 0.0;

	dst[8] = cvmGet(r, 0, 2);
	dst[9] = cvmGet(r, 1, 2);
	dst[10] = cvmGet(r, 2, 2);
	dst[11] = 0.0;

	dst[12] = cvmGet(t, 0, 0);
	dst[13] = cvmGet(t, 1, 0);
	dst[14] = cvmGet(t, 2, 0);
	dst[15] = 1.0;
}

void readwindow() 
{
	static int nframe = 0;
	IplImage *img = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), 8, 3);
	glReadPixels(0, 0, IMG_WIDTH, IMG_HEIGHT, GL_BGR_EXT, GL_UNSIGNED_BYTE, img->imageData); //! clearly do cast
	char str[100];
	cvFlip(img,img);
	sprintf_s(str,100, "screen/snap%d.jpg", nframe);
	cvSaveImage(str, img);
	cvReleaseImage(&img);
	nframe++;
}

void save(IplImage *templimg, char* name)
{
	char str[100];
	sprintf_s(str,100, "data/paperlist%s.txt", name);
	reg.ExtractRegions(templimg);
	m_tracker.CreateBlobSeq(&reg);
	m_tracker.SaveBlobSeq(str);
	reg.Clear();
	m_tracker.Clear();
	m_tracker.LoadBlobSeq(str);
	sprintf_s(str,100, "data/%s.jpg", name);
	cvSaveImage(str, templimg);
}

void process()
{
	reg.ExtractRegions(im);
	m_tracker.CreateBlobSeq(&reg);	
	m_tracker.FindBlobSeq(im);
	m_tracker.DrawBorder(bg);
	reg.Clear();
	m_tracker.ClearBlobSeq();
}

void setviewfromhomography(CvMat* homo)
{
	CvMat* intrinsic = cvCreateMat(3,3, MAT_TYPE);
	cvZero(intrinsic);

	double fx = 531.398682;
	double fy = 531.806702;
	double cx = 308.162262;
	double cy = 231.762756;

	cvmSet(intrinsic, 0,0, fx);
	cvmSet(intrinsic, 1,1, fy);
	cvmSet(intrinsic, 0,2, cx);
	cvmSet(intrinsic, 1,2, cy);
	cvmSet(intrinsic, 2,2, 1);			
	
	CvMat* inv =  cvCreateMat(3,3, MAT_TYPE);
	cvInvert(intrinsic, inv);
	
	CvMat* rdt0 =  cvCreateMat(3,3, MAT_TYPE);

	cvMatMul(inv, homo, rdt0);
	
	CvMat* rdt =  cvCreateMat(3,3, MAT_TYPE);
	
	double dvd = sqrt(cvmGet(rdt0,0, 0)*cvmGet(rdt0,0, 0)+cvmGet(rdt0,1, 0)*cvmGet(rdt0,1, 0)+cvmGet(rdt0,2, 0)*cvmGet(rdt0,2, 0));
	if (dvd==0)
		return;

	double norm = (double) ((double) 1.0/dvd);

	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			cvmSet(rdt, i,j, cvmGet(rdt0,i,j)*norm);
		}
	}
	
	CvMat *r1 = cvCreateMat(3,1, MAT_TYPE);
	CvMat *r2 = cvCreateMat(3,1, MAT_TYPE);
	CvMat *r1xr2 = cvCreateMat(3,1, MAT_TYPE);	
	
	for (int i=0;i<3;i++)
	{
		cvmSet(r1, i, 0, cvmGet(rdt,i, 0));
		cvmSet(r2, i, 0, cvmGet(rdt,i, 1));
	}
	
	cvCrossProduct(r1, r2, r1xr2);
	
	CvMat* R = cvCreateMat(3,3, MAT_TYPE);
	CvMat* t = cvCreateMat(3,1, MAT_TYPE);

	cvZero(R);
	cvZero(t);

	for(int y=0;y<2;y++){
		for(int x=0;x<3;x++){
			cvmSet(R, x,y, cvmGet(rdt,x,y));
		}
	}

	for(int x=0;x<3;x++){
		cvmSet(R, x,2,-cvmGet(r1xr2,x,0));
		cvmSet(t, 0,x,cvmGet(rdt,x,2));
	}
	
	setview(R, t, view);
	
	cvReleaseMat(&R);
	cvReleaseMat(&t);
	cvReleaseMat(&r1);
	cvReleaseMat(&r2);
	cvReleaseMat(&rdt);
	cvReleaseMat(&r1xr2);
	cvReleaseMat(&rdt0);
	cvReleaseMat(&inv);
	cvReleaseMat(&intrinsic);
}
	
void setprojection()
{
	double fx = 531.398682;
	double fy = 531.806702;
	double w = 640.0;
	double h = 480.0;
	double f = 10000.0;
	double n =-1; 
	projection[0] = 2*fx/w;
	projection[5] = -(2*fy/h);
	projection[10] = (f+n)/(f-n);
	projection[14] = 2*f*n/(f-n);
	projection[11] = 1;
}

void setlight()
{
	GLfloat mat_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_flash[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat mat_flash_shiny[] = { 70.0f };
	GLfloat lightZeroColor[] = { 0.9f, 0.0f, 0.0f, 1 };
	GLfloat light_position3[] = { 0.0f, 0.0f, 100.0f };

	glEnable(GL_TEXTURE_2D);
	glEnable (GL_LIGHTING);
	glEnable (GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position3);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
}

void drawCircle(float cx, float cy, float r, int num_segments) 
{ 
	glDisable (GL_LIGHTING);
	glEnable(GL_BLEND); //Enable blending.
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set blending function.

	glColor4f(1,0,0,0.5);
	glLineWidth(3);
	glBegin(GL_LINE_LOOP); 
	for(int ii = 0; ii < num_segments; ii++) 
	{ 
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle 
		float x = r * cosf(theta);//calculate the x component 
		float y = r * sinf(theta);//calculate the y component 
		glVertex2f(x + cx, y + cy);//output vertex 
	} 
	glEnd(); 
	glDisable(GL_BLEND);
	glEnable (GL_LIGHTING);
}

void drawText(float x, float y, float z, string str, float scale = 20, bool flip=false, bool rotated= false, float r=1, float g=1, float b=1)
{
	glDisable (GL_LIGHTING);
	
	glEnable(GL_BLEND); //Enable blending.
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //Set blending function.

	float lheight = 2;
	glPushMatrix();
	
	double hu = 20;
		
	glTranslated(x,y,z+rotated?hu:0);			
	
	if (rotated)
		glRotated(90, 1, 0,0);
	glScalef(scale, flip?-scale:scale,0);
	
	glColor4f(1,1,1,1);
	glPushMatrix();
	m_text.t3dDraw2D(str, 0, 0, lheight);
	glPopMatrix();
	glPopMatrix();	

	int w = (int)(str.length()*scale/2.3);
	int h = (int)(lheight*scale/2);
	glBegin(GL_QUADS);
	glColor4f(r,g,b,0.5);
	if (rotated)
	{
		glVertex3d(x-w,y,hu-h);
		glVertex3d(x+w,y,hu-h);
		glVertex3d(x+w,y,hu+h);
		glVertex3d(x-w,y,hu+h);

		glVertex3d(x-w/2,y,hu-h);
		glVertex3d(x,y,hu-h);
		glVertex3d(x,y,-h);
		glVertex3d(x,y,-h);
	}
	else
	{
		glVertex3f(x-w,y-h,0);
		glVertex3f(x+w,y-h,0);
		glVertex3f(x+w,y+h,0);
		glVertex3f(x-w,y+h,0);
	}
	glEnd();

	glDisable(GL_BLEND);
	glEnable (GL_LIGHTING);	
}

void drawcg()
{
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, IMG_WIDTH, 0, IMG_HEIGHT, -1.0, 1.0);

	list<blobseq*>::iterator lblob = m_tracker.lblobseqref.begin();
	for (;lblob!=m_tracker.lblobseqref.end();++lblob)
	{
		if ((*lblob)->overlap==0)
		{
				continue;
		}
						
		glEnable (GL_DEPTH_TEST);

		CvMat* hmt = cvCloneMat((*lblob)->homography);
		cvmSet(hmt,2, 0, cvmGet(hmt,2,0)/IMG_SCALE);
		cvmSet(hmt,2, 1, cvmGet(hmt,2,1)/IMG_SCALE);
		cvmSet(hmt,2, 2, cvmGet(hmt,2,2)/IMG_SCALE);
			
		setviewfromhomography(hmt);
		
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixd(projection);
		glMultMatrixd(view);

		glPushMatrix();			
		//draw labels
		for (unsigned i=0;i<(*lblob)->labels.size();i++)
		{
			glPushMatrix();
			drawCircle((float)((*lblob)->labels[i].x/IMG_SCALE), (float)((*lblob)->labels[i].y/IMG_SCALE), 3.0, 20);
			drawText((float)((*lblob)->labels[i].x/IMG_SCALE), (float)((*lblob)->labels[i].y/IMG_SCALE-10), 0, (*lblob)->labels[i].name, 7, !m_rotated, m_rotated, 1, 1, 0.0);
			glPopMatrix();	
		}
		drawText((float)((*lblob)->bound.x+(*lblob)->bound.width/2),(float)((*lblob)->bound.y+(*lblob)->bound.height/2), 0.0, (*lblob)->name, 20.0, true);

		glPopMatrix();
		glDisable(GL_DEPTH_TEST);
		cvReleaseMat(&hmt);
		(*lblob)->overlap=0;
	}
	
}

void close()
{		
	cvReleaseCapture(&cap);
	cvDestroyAllWindows();
	exit(1);
}

void display()
{
	im = cvQueryFrame(cap); //grab the frame from camera

	if (!im)
		close();

	if (m_save)
	{
		save(im, "online");
		m_save=false;
		return;
	} 

	if (!bg)
		bg = cvCloneImage(im);
	else
		cvCopy(im, bg);

	process(); //process the frame
	draw(wid, bg); //draw the background image
	drawcg(); //draw the opengl
	cvReleaseImage(&reg.gray);

	if (SAVE)
	{
		readwindow();
	}

	glutSwapBuffers();
	glFlush();
}

int main(int argc, char **argv) {

#ifdef WIN32
	SetPriorityClass( GetCurrentProcess(), REALTIME_PRIORITY_CLASS ); //! set priority to windows
#endif
	
	if (argc<2)
	{
		cout << "Usage : roadtrac [image file] [titlefile] [annotationfile]" << endl;
		cout << "exp : roadtrac template.jpg [label.txt] [annotations.txt]" << endl;
		exit(0);
	} else
	{
		IplImage *templ = cvLoadImage(argv[1]);
		reg.ExtractRegions(templ);	
		m_tracker.CreateBlobSeq(&reg);
		
		char paperlist[100];
		sprintf_s(paperlist,100, "paperlist.%s.txt", argv[1]);

		m_tracker.SaveBlobSeq(paperlist);
		reg.Clear();
		m_tracker.Clear();

		m_tracker.LoadBlobSeq(paperlist);

		if (argv[2])
			m_tracker.LoadBlobName(argv[2]);
		if (argv[3])
			m_tracker.LoadBlobLabels(argv[3]);
	}

	cap = cvCaptureFromCAM(CAMERAID);
	cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
	cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
	if (!cap)
		return 0;

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE| GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);

	glutInitWindowPosition(100, 100);
	glutInitWindowSize(IMG_WIDTH, IMG_HEIGHT);
	if (argv[1])
		wid = glutCreateWindow(argv[1]);
	else
		wid = glutCreateWindow("RoadTrac");

	texture = new GLuint[1];
	glGenTextures(1, texture);

	glBindTexture(GL_TEXTURE_2D, texture[0]);
	
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, FRAMESIZE, FRAMESIZE, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, 0);

	glutReshapeFunc(resize);
	glutDisplayFunc(display);
		
	m_text.t3dInit(DATA_CHARSET);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);

	setprojection();
			
	if (LIGHT) setlight();
	
	glutMainLoop();

	close();

	return 0;
}