#ifndef REGION_HPP
#define REGION_HPP

#include "opencv.h"
  
class Region{
	CvSeq* contours;
	int numcontour;

public:
	Region();
	~Region();

	void ExtractRegions(IplImage* col);
	void ExtractContour(CvSeq* cont);
	void Clear();
	int GetNum() {return numcontour;};

	list<blobseq*> lblobseq;
	IplImage *gray;
};

Region::Region()
{
}

Region::~Region()
{
}

void Region::Clear()
{
	lblobseq.clear();
}

void Region::ExtractRegions(IplImage* col)
{
	if (!col)
		return;

	IplImage *res = cvCreateImage( cvSize((int)(col->width/IMG_SCALE), (int)(col->height/IMG_SCALE)), IPL_DEPTH_8U, col->nChannels );
	cvResize(col, res);

	IplImage *grayImg = cvCreateImage( cvSize(res->width, res->height), IPL_DEPTH_8U, 1 );
	cvCvtColor(res, grayImg, CV_BGR2GRAY );
	cvReleaseImage(&res);

	cvAdaptiveThreshold(grayImg, grayImg, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 3,5);
	gray = cvCloneImage(grayImg);
	
	//find the contour
	CvSeq *cont;
	cvFindContours(grayImg, cvCreateMemStorage(), &cont, sizeof(CvContour), EXTERNAL?CV_RETR_EXTERNAL:CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1, cvPoint(0,0));
	
	if (!cont||cont->total<1)
	{
		cvReleaseImage(&grayImg);
		return;
	}

	numcontour=0;
	CvSeq* ct = cont;
		
	while (ct!=NULL)
	{
		ExtractContour(ct);
		ct = ct->h_next;
	}
	
	if (!numcontour)
		cvReleaseMemStorage(&cont->storage);

	cvReleaseImage(&grayImg);
}

//extracting the contour and produce the DCE contour
void Region::ExtractContour(CvSeq* cont)
{
	if (!cont)
		return;

	double area = cvContourArea(cont);
	
	CvSeq* oldcontour = cont;

	cont = cvApproxPoly(cont, sizeof(CvContour),cont->storage, CV_POLY_APPROX_DP , 3, 0);
		
	if ((cont->total<MINPOINTS&&LIMITREGION)||(area<MINAREA&&LIMITREGION))
	{
		return;
	}

	numcontour++;

	blobseq* bseq = new blobseq();
	bseq->id = NOID;
	cvReleaseMemStorage(&bseq->contours->storage);
	bseq->contours = cont;
	bseq->rawcountour = oldcontour;
	lblobseq.push_back(bseq);
}

#endif

