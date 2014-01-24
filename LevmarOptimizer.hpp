CvSeq *seq2,*seq1,*seq3;
#include "levmar.h"

void lmprojinv(double *p, double *x, int m, int n, void *data)
{
	double totdist = 0;
	int iter = seq2->total<LMRAWCONTOURN?1:(seq2->total/LMRAWCONTOURN); 
	for(int i = 0;i < n;i++){
		CvPoint*pt = (CvPoint*)cvGetSeqElem(seq2,i*iter);
		double d = p[6]*pt->x + p[7]*pt->y + p[8];
		double px = (p[0]*pt->x + p[1]*pt->y + p[2])/d;
		double py = (p[3]*pt->x + p[4]*pt->y + p[5])/d;
		double dist = abs(cvPointPolygonTest(seq1, cvPoint2D32f(px,py), 1));
		x[i] = dist;
		totdist+= dist;
	}
}

void lmproj(double *p, double *x, int m, int n, void *data)
{
	for(int i = 0;i < seq1->total;i++){
		CvPoint*pt = (CvPoint*)cvGetSeqElem(seq1,i);
		double d = p[6]*pt->x + p[7]*pt->y + 1.0;
		double px = (p[0]*pt->x + p[1]*pt->y + p[2])/d;
		double py = (p[3]*pt->x + p[4]*pt->y + p[5])/d;
		double dist=0.0;
		if (px>=0&&px<IMG_WIDTH&&py>=0&&py<IMG_HEIGHT)
		{				
			dist = abs(cvPointPolygonTest(seq2, cvPoint2D32f(px,py), 1));
		} 
		x[i] = dist;
	
	}

}

void lmprojneigh(double *p, double *x, int m, int n, void *data)
{
	for(int i = 0;i < seq1->total;i++){
		CvPoint*pt = (CvPoint*)cvGetSeqElem(seq1,i);
		double d = p[6]*pt->x + p[7]*pt->y + 1.0;
		double px = (p[0]*pt->x + p[1]*pt->y + p[2])/d;
		double py = (p[3]*pt->x + p[4]*pt->y + p[5])/d;
		CvPoint*p3 = (CvPoint*)cvGetSeqElem(seq3,i);
		double dist = sqrt((px-p3->x)*(px-p3->x)+(py-p3->y)*(py-p3->y));	
		x[i] = dist;
	}
}