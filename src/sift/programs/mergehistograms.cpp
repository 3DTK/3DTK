#include <fstream>
#include <iostream>


using namespace std;


int main (int argc, char const* argv[])
{

	ifstream *in[13];

	for (int i = 0 ; i < 13 ; i++) {
		in[i] = NULL;
	}


	in[2] = new ifstream("histogram_d-scan003.txt-scan008.txt_3600x1000");
	in[3] = new ifstream("histogram_d-scan006.txt-scan011.txt_3600x1000");
	in[4] = new ifstream("histogram_d-scan007.txt-scan000.txt_3600x1000");
	in[5] = new ifstream("histogram_d-scan005.txt-scan012.txt_3600x1000");
	
////	in[0] = new ifstream("histogram_r-scan001.txt-scan012.txt_2160x600");
////	in[1] = new ifstream("histogram_r-scan012.txt-scan000.txt_2160x600");
//	in[2] = new ifstream("histogram_r-scan001.txt-scan002.txt_2160x600");
//	in[3] = new ifstream("histogram_r-scan002.txt-scan003.txt_2160x600");
//	in[4] = new ifstream("histogram_r-scan003.txt-scan004.txt_2160x600");
//	in[5] = new ifstream("histogram_r-scan004.txt-scan005.txt_2160x600");
//	in[6] = new ifstream("histogram_r-scan005.txt-scan006a.txt_2160x600");
//	in[7] = new ifstream("histogram_r-scan006a.txt-scan007.txt_2160x600");
//	in[8] = new ifstream("histogram_r-scan007.txt-scan008.txt_2160x600");
//	in[9] = new ifstream("histogram_r-scan008.txt-scan009.txt_2160x600");
//	in[10] = new ifstream("histogram_r-scan009.txt-scan010.txt_2160x600");
////	in[11] = new ifstream("histogram_r-scan010.txt-scan011.txt_2160x600");
////	in[12] = new ifstream("histogram_r-scan011.txt-scan012.txt_2160x600");

	int length = 1000;
	

	double total = 0;

	int data[length];
	for (int i =0 ; i < length ; i++) {
		data[i] = 0;
	}
	

	for (int i = 0 ; i < 13 ; i++) {
		if (in[i])  {
			for (int j = 0; j < length; j += 1)
			{
				double ind;
				double val;
				*(in[i]) >> ind >> val;
				data[(int) (ind / 1 + 0.00001)] += val;
				total += val;
			}
			in[i]->close();
			delete in[i];
		}
	}

	ofstream out("histogram_d_s01_3600x1000");
	for (int i = 0; i < length; i += 1)
	{
//		out << (double) i * 2.0 << " " << (float) data[i] / total << endl;
		out << (double) i * 1.0 << " " << (float) data[i] << endl;
	}

	out.close();
	return 0;
}
