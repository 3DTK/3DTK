#include "riegl/scanlib.hpp"
#include <iostream>

using namespace std;
using namespace scanlib;

class Pointcloud : public pointcloud
{
public:
    Pointcloud() : pointcloud(false) {}
protected:
    void on_echo_transformed(echo_type echo) {
        pointcloud::on_echo_transformed(echo);
        target& t(targets[target_count-1]);
        cout << t.vertex[0] << ", " << t.vertex[1] << ", " << t.vertex[2] << ", " << t.time << endl;
    }
};

int main(int argc, char* argv[])
{
    if (argc != 2) { return -1; }

    shared_ptr<basic_rconnection> rc;
    rc = basic_rconnection::create(argv[1]);
    decoder_rxpmarker dec(rc);
    Pointcloud cloud;
    buffer buf;

    rc->open();

    for (dec.get(buf); !dec.eoi(); dec.get(buf)) {
        cloud.dispatch(buf.begin(), buf.end());
    }

    rc->close();

    return 0;
}
