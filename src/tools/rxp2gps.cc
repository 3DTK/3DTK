#include <iostream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <Eigen/Eigen>

#define private public
#define protected public

#include "riegl/scanlib.hpp"

using namespace std;
using namespace scanlib;


class GPS : public pointcloud
{
public:
    GPS()
        : pointcloud(true)
    {}
protected:
    void on_hk_gps_hr(const hk_gps_hr<iterator_type>& arg) {
        pointcloud::on_hk_gps_hr(arg);

        cout.precision(numeric_limits<double>::max_digits10);
        cout << (double) arg.TOWms * 1e-3 << "," << (double) arg.LAT * 1e-9 << "," << (double) arg.LONG * 1e-9 << "," << (double) arg.HGT_ELL * 1e-3 << endl;
    }
};

int main(int argc, char* argv[])
{
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("input,i", boost::program_options::value<string>(), "input folder")
            ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return EXIT_SUCCESS;
    }

    boost::filesystem::path dir(vm["input"].as<string>());

    for (boost::filesystem::directory_iterator it(dir); it != boost::filesystem::directory_iterator(); it++) {
        if (it->path().extension() != ".rxp") { continue; }

        string filename = it->path().string();

        cout << filename << endl;

        shared_ptr<basic_rconnection> rc;
        rc = basic_rconnection::create(filename);
        rc->open();
        decoder_rxpmarker dec(rc);
        buffer buf;

        GPS gps;

        for ( dec.get(buf); !dec.eoi() ; dec.get(buf) ) {
            gps.dispatch(buf.begin(), buf.end());
        }

        rc->close();

        cout << endl;
    }

    return 0;
}
