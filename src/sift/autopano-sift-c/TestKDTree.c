#include "AutoPanoSift.h"

// 2d example point class

struct sPoint3{
    IKDTreeDomain base;
    double* coords;
    double x;
    double y;
    double z;
};
typedef struct sPoint3 Point3;

int Point3_GetDimensionCount(Point3* self) {
    return (3);
}

int Point3_GetDimensionElement (Point3* self, int dim)
{
    return self->coords[dim];
}

Point3* Point3_new0()
{
    Point3* self = (Point3*)malloc(sizeof(Point3));
    self->base.getDimensionCount = (int ( *)(IKDTreeDomain *)) Point3_GetDimensionCount;
    self->base.getDimensionElement = (int ( *)(IKDTreeDomain *,int))Point3_GetDimensionElement;
    self->x = 0;
    self->y = 0;
    self->z = 0;
    self->coords = (double*)malloc(sizeof(double)*3);
    return self;
}


Point3*  Point3_new (double x, double y, double z)
{
    Point3* self = Point3_new0();
    self->x = x;
    self->y = y;
    self->z = z;
    self->coords[0]=x;
    self->coords[1]=y;
    self->coords[2]=z;
    return self;
}

void Point3_delete(Point3* self)
{
    free(self->coords);
    free(self);
}


Random* Point3_rnd = NULL;

Point3* Point3_CreateRandomPoint3 (double minX, double maxX,
				double minY, double maxY,
				double minZ, double maxZ)
{
    double x = Random_NextDouble (Point3_rnd) * (maxX - minX) + minX;
    double y = Random_NextDouble (Point3_rnd) * (maxY - minY) + minY;
    double z = Random_NextDouble (Point3_rnd) * (maxZ - minZ) + minZ;
    
    return (Point3_new (x, y, z));
}


int main (int argc, char* argv[])
{
    Point3 * target, *pt, *NN;
    KDTree* kd;
    double dist;
    int n;
    ArrayList* Point3s = ArrayList_new0 (Point3_delete);

    Point3_rnd = Random_new0 ();
    
    // try searching for a random target
    target = Point3_CreateRandomPoint3 (-100.0, 100.0, -100.0, 100.0, -100.0, 100.0);    
    WriteLine ("target is \nS %f %f %f", target->x, target->y, target->z);

    for ( n = 0 ; n < 20 ; ++n) {
	pt = Point3_CreateRandomPoint3 (-100.0, 100.0, -100.0, 100.0, -100.0, 100.0);
	WriteLine ("P %f %f %f  (distance: %f)", pt->x, pt->y, pt->z, sqrt((double)KDTree_DistanceSq((IKDTreeDomain *)pt, (IKDTreeDomain *)target)));
	ArrayList_AddItem (Point3s, pt);
    }
    
    WriteLine ("\nbuilding kd-tree");
    kd = KDTree_CreateKDTree (Point3s);
    
    WriteLine ("search for nearest neighbour");
    Point3* NN = (Point3*)KDTree_NearestNeighbour (kd, (IKDTreeDomain*)target, &dist);
    if (NN == NULL) {
	WriteLine ("no nn found");
    } else {
	WriteLine ("nearest is (distance %f):\nNN %f %f %f",
			   dist, NN->x, NN->y, NN->z);
    }


    {
	WriteLine ("search for 3 nearests neighbours");
	SortedLimitedList* nnlist = KDTree_NearestNeighbourList(kd, (IKDTreeDomain*)target, &dist, 3);
	WriteLine ("nearest distance is %f", dist);
	int i;
	for(i=0; i<SortedLimitedList_Count(nnlist); i++) {
	    KDTreeBestEntry* be = (KDTreeBestEntry*) SortedLimitedList_GetItem(nnlist, i);
	    Point3* nn = (Point3*)be->neighbour;
	    WriteLine ("N1 %f %f %f # distance %f",
		       nn->x, nn->y, nn->z, be->distance);
	}
	SortedLimitedList_delete(nnlist);
    }


    {
	WriteLine ("BBF search for 3 nearests neighbours");
	SortedLimitedList* nnlist = KDTree_NearestNeighbourListBBF(kd, (IKDTreeDomain*)target, 3, 20);
	int i;
	for(i=0; i<SortedLimitedList_Count(nnlist); i++) {
	    KDTreeBestEntry* be = (KDTreeBestEntry*) SortedLimitedList_GetItem(nnlist, i);
	    Point3* nn = (Point3*)be->neighbour;
	    WriteLine ("N2 %f %f %f # distance %f",
		       nn->x, nn->y, nn->z, be->distance);
	}
	SortedLimitedList_delete(nnlist);
    }


    Point3_delete(target);
    KDTree_delete(kd);
    ArrayList_delete(Point3s);
    Random_delete(Point3_rnd);

    return 0;
}


