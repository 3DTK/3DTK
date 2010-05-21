
#include "AutoPanoSift.h"

struct sPoint1 {
    double x;
};
typedef struct sPoint1 Point1;


void dump(ArrayList* l)
{
    int i;
    printf("[");
    for(i=0; i<ArrayList_Count(l); i++) {
	printf("%g ", ((Point1*)ArrayList_GetItem(l, i))->x);
    }
    printf("]\n");
}

Point1* Point1_new0()
{
    Point1* self = (Point1*)malloc(sizeof(Point1));
    self->x = 0;
    return self;
}


Point1*  Point1_new (double x)
{
    Point1* self = Point1_new0();
    self->x = x;
    return self;
}

void Point1_delete(Point1* self)
{
    if (self) {
	free(self);
    }
}

int Point1_Compare(IComparator* self, const Point1* p1, const Point1* p2) 
{
    if (p1->x<p2->x)
	return -1;
    else if(p1->x>p2->x)
	return 1;
    return 0;
}


int main(int argc, char* argv[])
{
    SortedLimitedList *s;
    IComparator cmp;
    Point1 *p;
    ArrayList* l = ArrayList_new0(Point1_delete);
    dump(l);
    ArrayList_AddItem(l, Point1_new(0));
    ArrayList_AddItem(l, Point1_new(1));
    ArrayList_AddItem(l, Point1_new(2));
    dump(l);
    // overwrite
    Point1_delete(ArrayList_GetItem(l,1));
    ArrayList_SetItem(l, 1, Point1_new(42));
    dump(l);
    Point1_delete(ArrayList_GetItem(l,1));
    ArrayList_RemoveAt(l, 1);
    dump(l);
    p = Point1_new(42);
    Point1_delete(ArrayList_GetItem(l,1));
    ArrayList_SetItem(l, 1, p);
    ArrayList_AddItem(l, Point1_new(2));
    dump(l);
    ArrayList_RemoveItem(l, p);
    Point1_delete(p);
    dump(l);
    ArrayList_AddItem(l, Point1_new(3));
    ArrayList_AddItem(l, Point1_new(4));
    ArrayList_AddItem(l, Point1_new(5));
    ArrayList_AddItem(l, Point1_new(6));
    dump(l);
    Point1_delete(ArrayList_GetItem(l,2));
    Point1_delete(ArrayList_GetItem(l,3));
    ArrayList_RemoveRange(l, 2, 2);
    dump(l);
    ArrayList_AddItem(l, Point1_new(3));
    ArrayList_AddItem(l, Point1_new(4));
    dump(l);
    
    cmp.compareTo = Point1_Compare;
    ArrayList_Sort(l, &cmp);
    dump(l);
    ArrayList_delete(l);

    s = SortedLimitedList_new(5, Point1_delete);
    s->comparator.compareTo = Point1_Compare;
    SortedLimitedList_AddItem(s, Point1_new(50));
    SortedLimitedList_AddItem(s, Point1_new(40));
    SortedLimitedList_AddItem(s, Point1_new(30));
    SortedLimitedList_AddItem(s, Point1_new(20));
    SortedLimitedList_AddItem(s, Point1_new(10));
    SortedLimitedList_AddItem(s, Point1_new(0));
    dump(&s->base);

    SortedLimitedList_AddItem(s, Point1_new(25));

    dump(&s->base);

    while (SortedLimitedList_Count(s)>0) {
	Point1_delete(SortedLimitedList_GetItem(s,0));
	SortedLimitedList_RemoveAt(s, 0);
	dump(&s->base);
    }
    SortedLimitedList_delete(s);
}
