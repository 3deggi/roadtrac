/**
 * Original Author:    Sandy Eggi
 * Created:   24.01.2014
 * 
 * Tracking the outlines using optimization method
 **/

#ifndef Tracker_HPP
#define Tracker_HPP

#include <iostream>
#include <string>
using namespace std;

#include "Blob.hpp"
#include "HashTable.hpp"
#include "Region.hpp"

class Tracker {
public:
	Tracker() {
	}

	~Tracker(){
	}

	void CreateBlobSeq(Region* reg);

	void SaveBlobSeq(const char* name) const;
	void LoadBlobSeq(const char* name);
	void LoadBlobName(const char* name);
	void LoadBlobLabels(const char* name);

	void FindBlobSeq(IplImage *im);
	void PostProcess(IplImage *im);
	void DrawSingleBorder(IplImage *im, blobseq *lblob);
	void ClearBlobSeq();
	void Update(blobseq *lblob);
	void Clear();

	list<blobseq*> lblobseq;	//current list of region
	list<blobseq*> lblobseqref; //database of region

	set<unsigned> detected;

	IplImage *grayImg;
private:
	HashTable m_tableseq;	//descriptor database
};


void Tracker::Clear()
{
	m_tableseq.Clear();
	lblobseqref.clear();
	detected.clear();
	ClearBlobSeq();
}

void Tracker::CreateBlobSeq(Region *reg)
{
	for (list<blobseq*>::iterator lb = reg->lblobseq.begin();lb!=reg->lblobseq.end();++lb)
	{
		lblobseq.push_back(*lb);
	}

	grayImg = reg->gray;
}

void Tracker::SaveBlobSeq(const char* name) const
{
	ofstream out(name);
	out.precision(10);
	out << static_cast<int>(lblobseq.size()) << endl;	//! num of contours

	int c = 0;
	for(list<blobseq*>::const_iterator lblob = lblobseq.begin(); lblob != lblobseq.end(); lblob++,c++){

	out << (*lblob)->contours->total << endl;	//! num of pts
	for (int i=0;i< (*lblob)->contours->total;i++)
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem((*lblob)->contours, i);
		out << p->x << " " <<p->y << endl;
	}
	out << endl;
	
	}
	out.close();
}

void Tracker::ClearBlobSeq() 
{
	if (lblobseq.size())
	{
		list<blobseq*>::iterator lblob = lblobseq.begin();
		CvMemStorage *st = (*lblob)->contours->storage;
		for(; lblob != lblobseq.end(); ++lblob){
			delete (*lblob);
		}
		if (st)
			cvReleaseMemStorage(&st);
		lblobseq.clear();
	}
}

void Tracker::LoadBlobSeq(const char* name) {
	
	ifstream in(name);
	int numblob;

	in >> numblob;

	for(unsigned short i=0;i<numblob;i++)
	{
		blobseq* bseq = new blobseq();
		bseq->id = i;
		
		int numpoints;
		in >> numpoints;

		for(int j=0;j<numpoints;j++)
		{			
			int x,y;
			in >> x >> y;
			CvPoint pt = cvPoint(x,y);			
			cvSeqPush(bseq->contours, &pt);
			cvSeqPush(bseq->prevcontours, &pt);
		}

		bseq->ComputeBoundingRect();
		bseq->ComputeMassCenter();
		bseq->curx = bseq->x;
		bseq->cury = bseq->y;
		bseq->ConstructRelevanceList();
		sprintf(bseq->name, "New map");

		m_tableseq.Init(bseq);
		lblobseqref.push_back(bseq);
		if (SINGLE)
			break;
	}
	m_tableseq.lpref = &lblobseqref;
	in.close();
}

void Tracker::LoadBlobName(const char* name) {
	
	ifstream in(name);
	if (!in)
		return;
	int numblob;

	in >> numblob;

	string str;
	getline(in,str);
	list<blobseq*>::iterator lblob = m_tableseq.lpref->begin();
	for (int i=0;i<numblob;lblob++,i++)
	{
		getline(in,str);
		strcpy((*lblob)->name, str.c_str());
		if (SINGLE)
			break;
	}
	in.close();
}

void Tracker::LoadBlobLabels(const char* name) {
	
	FILE *in = fopen(name, "r");
	
	if (!in)
		return;

	char str[50];
	fgets(str, 20, in);
	int numblob;
	sscanf(str, "%d", &numblob);
	
	list<blobseq*>::iterator lblob = m_tableseq.lpref->begin();
	for (int i=0;i<numblob;lblob++,i++)
	{
		int nlabels;
		fgets(str, 20, in);
		sscanf(str, "%d", &nlabels);
		for (int j=0;j<nlabels;j++)
		{
			fgets(str, 1024, in);
			char  lb[50];
			int x,y;
			sscanf(str, "%d %d %120[0-9a-zA-Z \-Q><^\E]s", &x, &y, lb);
			label lbl;
			lbl.x = x;
			lbl.y = y;
			strncpy(lbl.name, lb, 50);
			(*lblob)->labels.push_back(lbl);
		}
		if (SINGLE)
			break;
	}
	fclose(in);
}

void Tracker::FindBlobSeq(IplImage *im)
{
	double err = LMMAXERR;
	if (!im)
		return;

	if (!lblobseqref.size())
		return;

	map<unsigned, unsigned> ovl;
		
	for(list<blobseq*>::iterator lblob = lblobseq.begin(); lblob != lblobseq.end(); lblob++)
	{
		m_tableseq.MatchID((*lblob));
	
		if ((*lblob)->id!=NOID&&(*lblob)->id<lblobseqref.size())		//if found
		{				
			list<blobseq*>::iterator lref = lblobseqref.begin();
			advance(lref, (*lblob)->id);
			if (lref!=lblobseqref.end())
			{
				int num = 0;
				num = (*lblob)->ComputeHomography(*lref, im);
				if (num>MINOVERLAP) //if the pose estimation from correspondence is better than from the neighbor
				{
					//here should be removing points in the rawcontours
					if (CLEANRAW)
					(*lblob)->CleanRawContours((*lref));

					if (LMOPTIMIZENEIGH)
						(*lblob)->OptimizeHomographyPrev((*lref)->contours, (*lref)->prevcontours, (*lref)->homography);

					if (LMOPTIMIZE)
					{
						(*lblob)->OptimizeHomography((*lref)->contours,  LMRAWCONTOUR?(*lblob)->rawcountour:(*lblob)->contours, (*lblob)->homography, (*lref)->prevcontours);
						(*lblob)->overlap=(*lblob)->lmavgerror<err?(*lblob)->overlap:0;
						if (!ovl[(*lref)->id])
						{
							ovl[(*lref)->id] = (*lblob)->overlap;
							(*lref)->overlap = (*lblob)->overlap;
							cvCopy((*lblob)->homography, (*lref)->homography);

						} else
						{
							if (ovl[(*lref)->id]<(*lblob)->overlap)
							{
								(*lref)->overlap = (*lblob)->overlap;
								cvCopy((*lblob)->homography, (*lref)->homography);
								ovl[(*lref)->id] = (*lblob)->overlap;
							} 
						}
					}
				} else
					(*lblob)->overlap = 0;
				
			}
		}
	}

	//reset the unselected blob
	for(list<blobseq*>::iterator lblob = lblobseq.begin(); lblob != lblobseq.end(); lblob++)
	{
		if ((*lblob)->id!=NOID&&(*lblob)->id<lblobseqref.size())
		{
			if ((*lblob)->overlap<ovl[(*lblob)->id])
				(*lblob)->overlap=0;
		} else
				(*lblob)->overlap=0;
	}

	if (RECALLHIST)
	{
		set<unsigned>::iterator dt = detected.begin();
		for (;dt!=detected.end();++dt)
		{
			list<blobseq*>::iterator ref = lblobseqref.begin();
			if ((*dt)<(unsigned)lblobseqref.size())
			{
				unsigned id = (unsigned)(*dt);
				advance(ref, id);
				if ((*ref)->overlap) //check other previously detected
					continue;
				
				list<blobseq*>::iterator lblob = lblobseq.begin();
				bool found = false;
				for(; lblob != lblobseq.end(); lblob++)
				{
					unsigned oldov = (*lblob)->overlap;
					found = (*lblob)->ValidHomography(*ref, im, (*ref)->homography)>0;
					if (found&&(*lblob)->overlap>oldov)
						break;
				}
				if (found&&(lblob != lblobseq.end())) //if the pose estimation result overlap more than 0
				{
					if (LMOPTIMIZE)
					{
						(*lblob)->OptimizeHomography((*ref)->contours,  LMRAWCONTOUR?(*lblob)->rawcountour:(*lblob)->contours, (*ref)->homography, (*ref)->prevcontours);
						(*lblob)->overlap=(*lblob)->lmavgerror<err?(*lblob)->overlap:0;
					}  
				
					if ((*lblob)->overlap>0)
					{
						(*lblob)->id = id;
						cvCopy((*ref)->homography, (*lblob)->homography);
						(*ref)->overlap = (*lblob)->overlap;
						//insert to list of homography
						detected.insert((*ref)->id);
					}
				} 				
			}
		}
	}
}

void Tracker::DrawSingleBorder(IplImage *im, blobseq *lblob)
{
	if (!im)
		return;
	if (!lblob)
		return;

	if (lblob->id!=NOID&&lblob->overlap)
	{	
		list<blobseq*>::const_iterator ref = lblobseqref.begin();
		advance(ref, lblob->id);
		 
		CvRect rect = (*ref)->bound;
		
		double bo[12] = {
			rect.x, rect.x + rect.width, rect.x + rect.width, rect.x,
			rect.y, rect.y, rect.y + rect.height, rect.y + rect.height,
			1,      1,      1, 1,
		};
		CvMat mbo = cvMat(3, 4, MAT_TYPE, bo);
		CvMat *res = cvCreateMat(3, 4, MAT_TYPE);
		cvMatMul(lblob->homography, &mbo, res);

		for (int i=0;i<3;i++)
		{
			CvPoint p1 = cvPoint((int)(IMG_SCALE*cvmGet(res, 0, i)/cvmGet(res, 2, i)),(int)(IMG_SCALE*cvmGet(res, 1, i)/cvmGet(res, 2, i)));
			CvPoint p2 = cvPoint((int)(IMG_SCALE*cvmGet(res, 0, i+1)/cvmGet(res, 2, i+1)),(int)(IMG_SCALE*cvmGet(res, 1, i+1)/cvmGet(res, 2, i+1)));
			if (im->nChannels==4)
				cvLine(im, p1, p2, cvScalar(255,0,0, 255),1, CV_AA );
			else
			cvLine(im, p1, p2, CV_RGB(255,0,0),1, CV_AA );
		}
		CvPoint p1 =  cvPoint((int)(IMG_SCALE*cvmGet(res, 0, 0)/cvmGet(res, 2, 0)),(int)(IMG_SCALE*cvmGet(res, 1, 0)/cvmGet(res, 2, 0)));
		CvPoint p2 = cvPoint((int)(IMG_SCALE*cvmGet(res, 0, 3)/cvmGet(res, 2, 3)),(int)(IMG_SCALE*cvmGet(res, 1, 3)/cvmGet(res, 2, 3))); 
		if (im->nChannels==4)
				cvLine(im, p1, p2, cvScalar(255,0,0, 255),1, CV_AA );
		else
			cvLine(im, p1, p2, CV_RGB(255,0,0),1, CV_AA );		
		cvReleaseMat(&res);
		
	}		
}

void Tracker::Update(blobseq *lblob)
{
	if (!lblob)
		return;

	if (!lblob->Valid()||!lblob->Planarized())
		return;

	list<blobseq*>::iterator ref = lblobseqref.begin();
	if (lblob->id<lblobseqref.size())
	{
		blobseq * newb = new blobseq();
		advance(ref, lblob->id);
		newb->id = lblob->id;
		CvMat* mPt = cvCreateMat(3, (*ref)->contours->total, MAT_TYPE);
		for (int i=0;i<(*ref)->contours->total;i++)
		{
			CvPoint* p1 = CV_GET_SEQ_ELEM(CvPoint,(*ref)->contours,i);
			cvmSet(mPt, 0, i, p1->x);
			cvmSet(mPt, 1, i, p1->y);
			cvmSet(mPt, 2, i, 1);
		}
		CvMat* mRes = cvCreateMat(3, mPt->cols, MAT_TYPE);
		cvMatMul(lblob->homography, mPt, mRes);
		
		for (int i=0;i<mRes->cols;i++)
		{
				CvPoint p1 = cvPoint((int)(cvmGet(mRes, 0, i)/cvmGet(mRes, 2, i)),(int)(cvmGet(mRes, 1, i)/cvmGet(mRes, 2, i)));
			cvSeqPush(newb->contours, &p1);
		}
		
		newb->ConstructRelevanceList();
		newb->ComputeBoundingRect();
		newb->ComputeMassCenter();

		cvReleaseMat(&mRes);
		cvReleaseMat(&mPt);

		m_tableseq.Update(newb);
		CvMemStorage *st = newb->contours->storage;
		delete newb;
		cvReleaseMemStorage(&st);
	}
}

void Tracker::PostProcess(IplImage *im)
{
	if (!im)
		return;

	for(list<blobseq*>::const_iterator lblob = lblobseq.begin(); lblob != lblobseq.end(); lblob++)
	{		
		if ((*lblob)->id!=NOID&&(*lblob)->overlap)
			detected.insert((*lblob)->id);
		if (DRAWBORDER)
			DrawSingleBorder(im, (*lblob));
		if (UPDATE)
			Update(*lblob);
	}
}

#endif