#ifndef __GRIDPARAMS_H__
#define __GRIDPARAMS_H__

#include <cstdio>


class GridParams
{
public:
	double scale;
	double max_dist;

	GridParams()
	{
		num_buckets=128;
		scale=1;
		max_dist=1;
	}
	int buckets()
	{
		return num_buckets;
	}
	void buckets(int num)
	{
		if(num!=16 && num!=32 && num!=64 && num!=128 && num!=256 && num!=512)
		{
			printf("Wrong value (%d) for number of buckets. Choose from: 16,32,64,128,256,512\nWill use 128 instead!\n",num);
			num=128;
		}
		num_buckets=num;
	}

private:
	int num_buckets;

};

#endif
