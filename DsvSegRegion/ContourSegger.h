#include <vector>

#define MAXVALIDDIST	1.0

typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned int UINT;

typedef struct  
{
	int x,y;
	double opti;
}PTOpti;

typedef struct
{
	int x,y;
}IMCOORDINATE;


int FindFirstValidPt(int scanno);
void calcRectOpti();
void ContourExtraction();
UINT RegionGrow(bool withOpti);
void GrowOne(IMCOORDINATE seed, UINT regionID, int & xMin, int & xMax);
void Region2Seg ();
