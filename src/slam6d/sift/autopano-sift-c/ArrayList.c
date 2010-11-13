#include "AutoPanoSift.h"

/* This is a minimal subset of the C# Container Class ArrayList */

void ArrayList_init(ArrayList* self, void* deletefn)
{
    self->count = 0;
    self->reserved = 0;
    self->items = NULL;
    self->deletefn = deletefn;
}

ArrayList* ArrayList_new0(void* deletefn)
{
    ArrayList* self = (ArrayList*)malloc(sizeof(ArrayList));
    ArrayList_init(self, deletefn);
    return self;
}

ArrayList* ArrayList_new(int dim, void* deletefn)
{
    ArrayList* self = ArrayList_new0(deletefn);
    self->count = dim;
    self->reserved = dim;
    self->items = (void**)malloc(self->reserved * sizeof(void*));
    int index;
    for(index=0; index<self->reserved; index++) {
	self->items[index] = NULL;
    }    
    return self;
}

ArrayList* ArrayList_clone(ArrayList* self)
{
  ArrayList* cl = ArrayList_new0(self->deletefn);
  cl->count = self->count;
  cl->reserved = self->reserved;
  cl->items = (void**)malloc(cl->reserved * sizeof(void*));
  memcpy(cl->items, self->items, self->count * sizeof(void*));
  return cl;
}

void ArrayList_delete(ArrayList* self)
{
    if (self != NULL) {
	if (self->items != NULL) {
	    if (self->deletefn != NULL) {
		int index=0;
		void (*_deletefn)(void*) = (void ( *)(void *)) self->deletefn;
		for(index=0; index<self->count; index++) {
		    if (self->items[index]) {
			_deletefn(self->items[index]);
			self->items[index] = NULL;
		    }
		}
	    }
	    free(self->items);
	    self->items = NULL;
	}
	self->count = 0;
	self->reserved = 0;
	free(self);
    }
}

int ArrayList_Count(ArrayList* self)
{
    return self->count;
}

void* ArrayList_GetItem(ArrayList* self, int index)
{
    if (index < 0 || index >= self->count) {
	FatalError("Array out of bounds");
	return NULL;
    }
    return self->items[index];
}

void ArrayList_SetItem(ArrayList* self, int index, void* item)
{
    if (index < 0 || index >= self->count) {
	WriteError("Array out of bounds");
	return;
    }
    self->items[index] = item;
}

int ArrayList_IndexOf(ArrayList* self, void* item)
{
    int index;
    for(index=0; index<self->count; index++) {
	if( self->items[index] == item) {
	    return index;
	}
    }
    return -1;
}

bool ArrayList_Contains(ArrayList* self, void* item)
{
    return ArrayList_IndexOf(self, item) >= 0;
}

void ArrayList_AddItem(ArrayList* self, void* item)
{
    if (self->count+1 > self->reserved) {
	void* tmp = self->items;
	self->reserved = self->reserved * 5 + 10;
	self->items = (void**)malloc(self->reserved * sizeof(void*));
	if (tmp != NULL) {
	    memcpy(self->items, tmp, self->count * sizeof(void*));
	    free(tmp);
	}
    }
    self->items[self->count] = item;
    self->count++;
}

void ArrayList_AddRange(ArrayList* self, ArrayList* list)
{
    if (self->count+list->count > self->reserved) {
	void* tmp = self->items;
	self->reserved = self->count + list->count + 10;
	self->items = (void**)malloc(self->reserved * sizeof(void*));
	if (tmp != NULL) {
	    memcpy(self->items, tmp, self->count * sizeof(void*));
	    free(tmp);
	}
    }
    memcpy(self->items+self->count, list->items, list->count * sizeof(void*));
    self->count += list->count;
}

void ArrayList_Copy(ArrayList* self, int start, ArrayList* dest, int offset, int len)
{
    if (dest->reserved < len) {
	if (dest->reserved>0) {
	    free(dest->items);
	}
	dest->reserved = len+10;
	dest->items = (void**)malloc(dest->reserved * sizeof(void*));
    }
    memcpy(dest->items, self->items+start, len*sizeof(void*));
}


void ArrayList_RemoveItem(ArrayList* self, void* value)
{
    int index;
    for(index=0; index<self->count; index++) {
	if (self->items[index] == value) {
	    memmove(self->items+index, self->items+index+1, (self->count-(index+1))*sizeof(void*));
	    self->count--;
	    return;
	}
    }
}

void ArrayList_RemoveAt(ArrayList* self, int index)
{
    memmove(self->items+index, self->items+index+1, (self->count-(index+1))*sizeof(void*));
    self->count--;
}

void ArrayList_RemoveRange(ArrayList* self, int start, int len)
{
    memmove(self->items+start, self->items+start+len, (self->count-(start+len))*sizeof(void*));
    self->count-=len;
}

IComparator* ArrayList_comparator = NULL;

int ArrayList_compareTo(const void** obj1, const void** obj2) {
    return ArrayList_comparator->compareTo(ArrayList_comparator, *obj1, *obj2);
}

void ArrayList_Sort(ArrayList* self, IComparator* cmp) {
    if (ArrayList_comparator != NULL) {
	FatalError("ArrayList_Sort called recursively");
    }
    ArrayList_comparator = cmp;
    qsort(self->items, self->count, sizeof(void*), (int ( *)(const void *,const void *)) ArrayList_compareTo);
    ArrayList_comparator = NULL;    
}
