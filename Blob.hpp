/**
 * Original Author:    Sandy Eggi
 * Created:   24.01.2014
 * 
 * Blob/contour manipulation, including optimization
 **/

#ifndef MYBLOB_HPP
#define MYBLOB_HPP

#include <cmath>
#include <vector>
#include <map>
#include <list>
using namespace std;

#include "opencv.h"
#include "levmarOptimizer.hpp"

const unsigned NOID = 0xffffffff;

struct label
{
		char name[50];
		int x,y;
};

typedef pair<CvPoint,CvPoint> ppoint; 

struct blobseq
{
	static double RelevanceMeasure(CvPoint p1, CvPoint p2, CvPoint p3);

	int ComputeHomography(blobseq* ref, IplImage *im);
	int ValidHomography(blobseq* ref, IplImage *im, CvMat* homography); //modified

	void Clear(); //clear and reset the contents of relevance list
	void ComputeMassCenter(); //compute the mass of the contour using moment
	void ComputeBoundingRect(); //compute the bounding box of the contour
	void InsertRelevance(unsigned rel); //insert a value to relevance list
	void ConstructRelevanceList(); //compute the relevance measure and insert into the relevance list
	
	bool Valid() const { return (id != NOID); } //valid if the blob is detected
	bool Planarized() const {return overlap>0;} //planarized if the value of overlap is positive
	
	void OptimizeHomography(CvSeq* cont1, CvSeq* cont2, CvMat* inithomo, CvSeq* cont3=NULL );
	void OptimizeHomographyPrev(CvSeq* cont1, CvSeq* cont2, CvMat* inithomo);
	void CleanRawContours(blobseq* ref);

	unsigned id;
	unsigned overlap;
	int x,y;				//the gravity center
	int curx,cury;
	double weight;
	double lmavgerror;
	char name[50];
	CvRect bound;

	CvMat* homography;
	CvSeq* contours;
	CvSeq* rawcountour;
	CvSeq* prevcontours;
	list<unsigned> relevance;
	map<unsigned, unsigned> candidates;
	list<pair<unsigned,unsigned> > match;
	vector<label> labels;

	blobseq();
	~blobseq();
};

blobseq::blobseq()
{
	homography = cvCreateMat(3,3, MAT_TYPE);
	cvZero(homography);
	contours = cvCreateSeq(CV_SEQ_POLYGON, sizeof(CvSeq), sizeof(CvPoint), cvCreateMemStorage());
	prevcontours = cvCreateSeq(CV_SEQ_POLYGON, sizeof(CvSeq), sizeof(CvPoint), cvCreateMemStorage());
}

blobseq::~blobseq()
{
	Clear();
	match.clear();
	relevance.clear();
	if (contours)
	{
		cvClearSeq(contours);
	}
	cvReleaseMemStorage(&prevcontours->storage);
	cvReleaseMat(&homography);
}

void blobseq::ComputeBoundingRect()
{
	if (!contours)
		return;
	bound = cvBoundingRect(contours);
}

void blobseq::ComputeMassCenter()
{
	if (!contours)
		return;
	CvMoments m_moments;
	cvMoments( contours, &m_moments);
	x = (int)(m_moments.m10/m_moments.m00);
	y = (int)(m_moments.m01/m_moments.m00);
}

void blobseq::InsertRelevance(unsigned rel)
{
	relevance.push_back(rel);
}

void blobseq::Clear()
{
	id = 0;
	weight = 0.0;
}

double blobseq::RelevanceMeasure(CvPoint p1, CvPoint p2, CvPoint p3)
{
	CvPoint v1 = cvPoint((p2.x-p1.x), (p2.y-p1.y));//more stable than abs
	CvPoint v2 = cvPoint((p3.x-p2.x), (p3.y-p2.y));
	double l1 = (sqrt(1.0*v1.x*v1.x+v1.y*v1.y));
	if (l1<=0)
		return 0;
	double l2 = (sqrt(1.0*v2.x*v2.x+v2.y*v2.y));
	if (l2<=0)
		return 0;
	double val = 1.0*(v1.x*v2.x+v1.y*v2.y)/(l1*l2);
	double angle = 0;
	if (abs(val)>1) return 0;
	angle = acos(val)*180/CV_PI;
	return (angle*l1*l2/(l1*l1+l2*l2));
}

void blobseq::ConstructRelevanceList()
{
	if (!contours)
		return;
	relevance.clear();

	for (int r=0;r<contours->total-2;r++)
	{
		CvPoint* p1 = CV_GET_SEQ_ELEM(CvPoint,contours,r);
		CvPoint* p2 = CV_GET_SEQ_ELEM(CvPoint,contours,r+1);
		CvPoint* p3 = CV_GET_SEQ_ELEM(CvPoint,contours,r+2);
		double val = RelevanceMeasure(*p1,*p2,*p3);

		if (CAST)
		{
			unsigned base = (unsigned)floor(val);
			if (val-base>=0.5)
				base +=1;
			
			InsertRelevance(base);

		} else
			InsertRelevance((unsigned)val);
	}

	CvPoint* p1 = CV_GET_SEQ_ELEM(CvPoint,contours,contours->total-2);
	CvPoint* p2 = CV_GET_SEQ_ELEM(CvPoint,contours,contours->total-1);
	CvPoint* p3 = CV_GET_SEQ_ELEM(CvPoint,contours,0);
	double val = RelevanceMeasure(*p1,*p2,*p3);

	if (CAST)
	{
		unsigned base = (unsigned)floor(val);
		if (val-base>=0.5)
			base +=1;
		InsertRelevance(base);

	} else
		InsertRelevance((unsigned)val);

	p1 = CV_GET_SEQ_ELEM(CvPoint,contours,contours->total-1);
	p2 = CV_GET_SEQ_ELEM(CvPoint,contours,0);
	p3 = CV_GET_SEQ_ELEM(CvPoint,contours,1);
	val = RelevanceMeasure(*p1,*p2,*p3);

	if (CAST)
	{
		unsigned base = (unsigned)floor(val);
		if (val-base>=0.5)
			base +=1;
		InsertRelevance(base);
	} else
		InsertRelevance((unsigned)val);

}

int blobseq::ComputeHomography(blobseq* ref, IplImage *im)
{
	if (match.size()<MINMATCH)
		return 0;	

	bool res = false;

	CvMat *m1 = cvCreateMat(2, (int)match.size(), MAT_TYPE);
	CvMat *m2 = cvCreateMat(2, (int)match.size(), MAT_TYPE);


	int i = 0;
	for (list<pair<unsigned,unsigned> >::iterator corid =  match.begin();corid!=match.end();++corid,i++)
	{
		unsigned id1 = (*corid).first;
		unsigned id2 = (*corid).second;

		CvPoint *pt1 = CV_GET_SEQ_ELEM(CvPoint,ref->contours,id1);
		CvPoint *pt2 = CV_GET_SEQ_ELEM(CvPoint,contours,id2);
	
		if (!pt1||!pt2)
			continue;

		cvmSet(m1, 0, i, pt1->x);
		cvmSet(m1, 1, i, pt1->y);
		cvmSet(m2, 0, i, pt2->x);
		cvmSet(m2, 1, i, pt2->y);
	}
	
	cvFindHomography(m1,m2, homography, CV_RANSAC, 3);

	cvReleaseMat(&m2);
	cvReleaseMat(&m1);

	overlap = ValidHomography(ref, im, homography);

	return overlap;
}

void  blobseq::OptimizeHomography(CvSeq* cont1, CvSeq* cont2, CvMat* inithomo, CvSeq* cont3)
{

	int n;

	seq1 = cont1;
	seq2 = cont2;
	
	double *x;
	int m;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;opts[4]= LM_DIFF_DELTA; 

	lmavgerror = 0.0;

	if (LMOPTIMIZEINV)
	{
		CvMat *invh = cvCreateMat(3,3, MAT_TYPE);
		cvInvert(inithomo, invh);
		
		double p[9];
		m=9; 
		//n=cont2->total;
		n=cont2->total<LMRAWCONTOURN?cont2->total:LMRAWCONTOURN;
		x = (double*)malloc(n*sizeof(double));
		

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
			{
				p[i*3+j] = cvmGet(invh, i, j);
			} 

		for (int i=0;i<n;i++)
			x[i] = 0.0;
		
		int iterres = dlevmar_dif(lmprojinv, p, x, m, n, LMITER, opts, info, NULL, NULL, NULL); // no Jacobian, caller allocates work memory, covariance estimated

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				cvmSet(invh, i, j, p[i*3+j]);

		for (int i=0;i<n;i++){
			lmavgerror+= x[i];
		}
		lmavgerror/=n;

		cvInvert(invh, inithomo);
		cvReleaseMat(&invh);

	}
	else
	{
		double p[8];
		m=8; n=cont1->total;
		x = (double*)malloc(n*sizeof(double));

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
			{
				if (i*3+j<8)
				p[i*3+j] = cvmGet(inithomo, i, j);
			}

		for (int i=0;i<n;i++)
			x[i] = 0.0;
		
		dlevmar_dif(lmproj, p, x, m, n, LMITER, opts, info, NULL, NULL, NULL); // no Jacobian, caller allocates work memory, covariance estimated

		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++)
				if (i*3+j<8)
				cvmSet(inithomo, i, j, p[i*3+j]);

		for (int i=0;i<n;i++)
			lmavgerror+= x[i];
		lmavgerror/=n;
	}
}

void  blobseq::OptimizeHomographyPrev(CvSeq* cont1, CvSeq* cont2, CvMat* inithomo)
{
  #if !defined(__GNUC__)
  	
	seq1 = cont1;
	seq3 = cont2;

	double *x;
	int m, n;
	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;opts[4]= LM_DIFF_DELTA; 

	double p[8];
	m=8; n=cont1->total;

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
		{
			if (i*3+j<8)
			p[i*3+j] = cvmGet(inithomo, i, j);
		}

	double av = 0.0;
	for(int i = 0;i < seq1->total;i++){
		CvPoint*pt = (CvPoint*)cvGetSeqElem(seq1,i);
		double d = p[6]*pt->x + p[7]*pt->y + 1.0;
		double px = (p[0]*pt->x + p[1]*pt->y + p[2])/d;
		double py = (p[3]*pt->x + p[4]*pt->y + p[5])/d;
		CvPoint*p3 = (CvPoint*)cvGetSeqElem(seq3,i);
		av+= sqrt((px-p3->x)*(px-p3->x)+(py-p3->y)*(py-p3->y));	
	}
	if (av/seq1->total>MAXNEIGHDIST)
		return;

	x = (double*)malloc(n*sizeof(double));
	for (int i=0;i<n;i++)
		x[i] = 0.0;
	
	dlevmar_dif(lmprojneigh, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // no Jacobian, caller allocates work memory, covariance estimated

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			if (i*3+j<8)
			cvmSet(inithomo, i, j, p[i*3+j]);
  
  #endif
}

void blobseq::CleanRawContours(blobseq* ref)
{
	CvMat* mPt = cvCreateMat(3, rawcountour->total, MAT_TYPE);
	for (int i=0;i<rawcountour->total;i++)
	{
		CvPoint* p1 = CV_GET_SEQ_ELEM(CvPoint,rawcountour,i);
		cvmSet(mPt, 0, i, p1->x);
		cvmSet(mPt, 1, i, p1->y);
		cvmSet(mPt, 2, i, 1);
	}
	CvMat* mRes = cvCreateMat(3, mPt->cols, MAT_TYPE);
	CvMat * hm = cvCreateMat(3,3, MAT_TYPE);
	cvInvert(homography,hm);
	cvMatMul(hm, mPt, mRes);
	
	for (int i=rawcountour->total-1;i>=0;i--)
	{
		CvPoint p1 = cvPoint((int)(cvmGet(mRes, 0, i)/cvmGet(mRes, 2, i)),(int)(cvmGet(mRes, 1, i)/cvmGet(mRes, 2, i)));

		cvmSet(mRes, 0, i, cvmGet(mRes, 0, i)/cvmGet(mRes, 2, i));
		cvmSet(mRes, 1, i, cvmGet(mRes, 1, i)/cvmGet(mRes, 2, i));
		cvmSet(mRes, 2, i, 1);

		if (abs(cvPointPolygonTest(ref->contours, cvPoint2D32f(p1.x, p1.y), 1))>2*DISTTOPOLY) //remove contours for handling occlusion
			cvSeqRemove(rawcountour, i);	
	}

	cvReleaseMat(&hm);
	cvReleaseMat(&mRes);
	cvReleaseMat(&mPt);	
}

int blobseq::ValidHomography(blobseq* ref, IplImage *im, CvMat* homography)
{
	CvMat* mPt = cvCreateMat(3, ref->contours->total, MAT_TYPE);
	for (int i=0;i<ref->contours->total;i++)
	{
		CvPoint* p1 = CV_GET_SEQ_ELEM(CvPoint,ref->contours,i);
		cvmSet(mPt, 0, i, p1->x);
		cvmSet(mPt, 1, i, p1->y);
		cvmSet(mPt, 2, i, 1);
	}
	CvMat* mRes = cvCreateMat(3, mPt->cols, MAT_TYPE);
	cvMatMul(homography, mPt, mRes);
	
	CvSeq* proj = cvCreateSeq(CV_SEQ_POLYGON, sizeof(CvSeq), sizeof(CvPoint), cvCreateMemStorage());

	int dh = 0;
	for (int i=0;i<ref->contours->total;i++)
	{
		CvPoint p1 = cvPoint((int)(cvmGet(mRes, 0, i)/cvmGet(mRes, 2, i)),(int)(cvmGet(mRes, 1, i)/cvmGet(mRes, 2, i)));
		cvSeqPush(proj, &p1);

		if (abs(cvPointPolygonTest(contours, cvPoint2D32f(p1.x, p1.y), 1))<DISTTOPOLY) //this wont work if there is double edge
			dh++;
	}
	cvClearSeq(proj);
	cvReleaseMemStorage(&proj->storage);

	overlap = dh;

	cvReleaseMat(&mRes);
	cvReleaseMat(&mPt);

	bool test = dh>ref->contours->total*CLOSEPOINTS;
	return (test?dh:0);
}

#endif