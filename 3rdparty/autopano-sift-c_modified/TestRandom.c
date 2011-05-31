#include "AutoPanoSift.h"

//#ifdef TEST_MAIN
int main(int argc, char* argv[]) {
    Random* r = Random_new0();
    int i;
    for(i=0; i<20; i++) {
	printf("rand(%d, %d)=%d\n", 10, 42, Random_Next(r, 10, 42));
    }
    Random_delete(r);
    return 0;
}
//#endif
