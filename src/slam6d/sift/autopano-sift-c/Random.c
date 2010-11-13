#include "AutoPanoSift.h"
#include <math.h>
#include <time.h>

/* This is a minimal subset of the C# Container Class Random */

Random* Random_new0() {
    Random* self = (Random*)malloc(sizeof(Random));
    srand(time(NULL));
    return self;
}

void Random_delete(Random* self) {
    free(self);
}

int Random_Next(Random* self, int min, int max) {
    return min + (int)((rand()*(double)(max-min))/RAND_MAX);
}

double Random_NextDouble(Random* self) {
    return ((double)(rand())/RAND_MAX);
}

