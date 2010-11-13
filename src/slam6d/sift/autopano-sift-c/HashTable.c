#include "AutoPanoSift.h"

/* This is a minimal subset of the C# Container Class Hashtable */

/* At the moment, there is NO hashtable functionality...
** just an implementation of a similar interface
** As far as I can see, the Hashtables are small and little used
** so no  performance issue expected from this braindead implementation
*/ 

HashTable* HashTable_new0(void* delete_key, void* delete_value)
{
    HashTable* self = (HashTable*)malloc(sizeof(HashTable));
    self->keys = ArrayList_new0(delete_key);
    self->values = ArrayList_new0(delete_value);
    return self;
}

void HashTable_delete(HashTable* self)
{
    if (self) {
	ArrayList_delete(self->keys);
	ArrayList_delete(self->values);
	free(self);
    }
}

void* HashTable_GetItem(HashTable* self, void* key)
{
    int index = ArrayList_IndexOf(self->keys, key);
    if (index < 0) {
	return NULL;
    }
    return ArrayList_GetItem(self->values, index);
}

void HashTable_SetItem(HashTable* self, void* key, void* value)
{
    int index = ArrayList_IndexOf(self->keys, key);
    if (index >= 0) {
	ArrayList_SetItem(self->values, index, value);
    } else {
	HashTable_AddItem(self, key, value);
    }
}

void HashTable_AddItem(HashTable* self, void* key, void* value)
{
    ArrayList_AddItem(self->keys, key);
    ArrayList_AddItem(self->values, value);
}

bool HashTable_Contains(HashTable* self, void* key)
{
    return ArrayList_Contains(self->keys, key);
}
