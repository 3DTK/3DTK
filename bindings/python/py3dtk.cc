#include "slam6d/scan.h"
#include "slam6d/basicScan.h"
#include "slam6d/kdIndexed.h"
#include "slam6d/kd.h"
#include "slam6d/normals.h"
#include "spherical_quadtree/spherical_quadtree.h"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#if PY_MAJOR_VERSION != 3
	#error require python3
#endif

namespace bp = boost::python;

DataPointer (Scan::*scan_getByString)(const std::string&) = &Scan::get;
void (Scan::*scan_getByType)(IODataType) = &Scan::get;

bp::list QuadTree_search(QuadTree &tree, bp::tuple _p, double r)
{
	double *_pv = new double[3];
	_pv[0] = bp::extract<double>(_p[0]);
	_pv[1] = bp::extract<double>(_p[1]);
	_pv[2] = bp::extract<double>(_p[2]);
	std::vector<size_t> res = tree.search(_pv, r);
	delete[] _pv;
	bp::list l;
	for (auto &it: res) {
		l.append(it);
	}
	return l;
}

bp::tuple scan_get_rPos(Scan &s)
{
	const double *rPos = s.get_rPos();
	bp::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(rPos[i]);
	}
	return bp::tuple(l);
}

bp::tuple scan_get_rPosTheta(Scan &s)
{
	const double *rPosTheta = s.get_rPosTheta();
	bp::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(rPosTheta[i]);
	}
	return bp::tuple(l);
}

bp::tuple scan_get_transMatOrg(Scan &s)
{
	const double *transMatOrg = s.get_transMatOrg();
	bp::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(transMatOrg[i]);
	}
	return bp::tuple(l);
}

void scan_transform(Scan &s, bp::tuple m, const Scan::AlgoType type, bool islum = 0)
{
	const double matrix[16] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
		bp::extract<double>(m[9])(),
		bp::extract<double>(m[10])(),
		bp::extract<double>(m[11])(),
		bp::extract<double>(m[12])(),
		bp::extract<double>(m[13])(),
		bp::extract<double>(m[14])(),
		bp::extract<double>(m[15])()
	};
	s.transform(matrix, type, islum);
}

void scan_transformAll(Scan &s, bp::tuple m)
{
	const double matrix[16] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
		bp::extract<double>(m[9])(),
		bp::extract<double>(m[10])(),
		bp::extract<double>(m[11])(),
		bp::extract<double>(m[12])(),
		bp::extract<double>(m[13])(),
		bp::extract<double>(m[14])(),
		bp::extract<double>(m[15])()
	};
	s.transformAll(matrix);
}

bp::tuple pyM4identity()
{
	bp::list l;
	l.append(1.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(1.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(1.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(0.0f);
	l.append(1.0f);
	return bp::tuple(l);
}

bp::tuple pytransform3(bp::tuple m, bp::tuple p)
{
	const double matrix[16] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
		bp::extract<double>(m[9])(),
		bp::extract<double>(m[10])(),
		bp::extract<double>(m[11])(),
		bp::extract<double>(m[12])(),
		bp::extract<double>(m[13])(),
		bp::extract<double>(m[14])(),
		bp::extract<double>(m[15])()
	};
	double point[3] = {
		bp::extract<double>(p[0])(),
		bp::extract<double>(p[1])(),
		bp::extract<double>(p[2])(),
	};
	transform3(matrix, point);
	bp::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(point[i]);
	}
	return bp::tuple(l);
}

bp::tuple pytransform3normal(bp::tuple m, bp::tuple n)
{
	const double matrix[16] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
		bp::extract<double>(m[9])(),
		bp::extract<double>(m[10])(),
		bp::extract<double>(m[11])(),
		bp::extract<double>(m[12])(),
		bp::extract<double>(m[13])(),
		bp::extract<double>(m[14])(),
		bp::extract<double>(m[15])()
	};
	double normal[3] = {
		bp::extract<double>(n[0])(),
		bp::extract<double>(n[1])(),
		bp::extract<double>(n[2])(),
	};
	transform3normal(matrix, normal);
	bp::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(normal[i]);
	}
	return bp::tuple(l);
}

bp::tuple pyEulerToMatrix3(bp::tuple t)
{
	const double rPosTheta[3] = {
		bp::extract<double>(t[0])(),
		bp::extract<double>(t[1])(),
		bp::extract<double>(t[2])(),
	};
	double matrix[9];
	EulerToMatrix3(rPosTheta, matrix);
	bp::list l;
	for (int i = 0; i < 9; ++i) {
		l.append(matrix[i]);
	}
	return bp::tuple(l);
}

bp::tuple pyEulerToMatrix4(bp::tuple p, bp::tuple t)
{
	const double rPos[3] = {
		bp::extract<double>(p[0])(),
		bp::extract<double>(p[1])(),
		bp::extract<double>(p[2])(),
	};
	const double rPosTheta[3] = {
		bp::extract<double>(t[0])(),
		bp::extract<double>(t[1])(),
		bp::extract<double>(t[2])(),
	};
	double matrix[16];
	EulerToMatrix4(rPos, rPosTheta, matrix);
	bp::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(matrix[i]);
	}
	return bp::tuple(l);
}

bp::tuple pyM3inv(bp::tuple m)
{
	const double Min[9] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
	};
	double Mout[9];
	M3inv(Min, Mout);
	bp::list l;
	for (int i = 0; i < 9; ++i) {
		l.append(Mout[i]);
	}
	return bp::tuple(l);
}

bp::tuple pyM4inv(bp::tuple m)
{
	const double Min[16] = {
		bp::extract<double>(m[0])(),
		bp::extract<double>(m[1])(),
		bp::extract<double>(m[2])(),
		bp::extract<double>(m[3])(),
		bp::extract<double>(m[4])(),
		bp::extract<double>(m[5])(),
		bp::extract<double>(m[6])(),
		bp::extract<double>(m[7])(),
		bp::extract<double>(m[8])(),
		bp::extract<double>(m[9])(),
		bp::extract<double>(m[10])(),
		bp::extract<double>(m[11])(),
		bp::extract<double>(m[12])(),
		bp::extract<double>(m[13])(),
		bp::extract<double>(m[14])(),
		bp::extract<double>(m[15])()
	};
	double Mout[16];
	M4inv(Min, Mout);
	bp::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(Mout[i]);
	}
	return bp::tuple(l);
}

// given a DataXYZ and an index, assemble a Python tuple to return
// that contains the xyz data
bp::tuple DataXYZ_getitem(DataXYZ &s, size_t index)
{
	if (index >= 0 && index < s.size()) {
		bp::list l;
		for (int i = 0; i < 3; ++i) {
			l.append(s[index][i]);
		}
		return bp::tuple(l);
	} else {
		PyErr_SetString(PyExc_IndexError, "index out of range");
		bp::throw_error_already_set();
		// because of the thrown exception, this will never be reached
		// we do this to make the compiler happy
		return bp::tuple();
	}
}

size_t DataXYZ_length(DataXYZ &s)
{
	return s.size();
}

float DataReflectance_getitem(DataReflectance &s, size_t index)
{
	if (index >= 0 && index < s.size()) {
		return s[index];
	} else {
		PyErr_SetString(PyExc_IndexError, "index out of range");
		bp::throw_error_already_set();
		// because of the thrown exception, this will never be reached
		// we do this to make the compiler happy
		return 0.0f;
	}
}

size_t DataReflectance_length(DataReflectance &s)
{
	return s.size();
}

// we need to wrap KDtreeIndexed because its constructor takes a double**
// which boost python cannot handle directly
class KDtreeIndexedWrapper : public KDtreeIndexed
{
	public:
		KDtreeIndexedWrapper(bp::list l) : KDtreeIndexed()
		{
			size_t len = bp::extract<std::size_t>(l.attr("__len__")());
			double** pa = new double*[len];
			for (size_t i = 0; i < len; ++i) {
				bp::tuple t = bp::extract<bp::tuple>(l[i]);
				pa[i] = new double[3];
				pa[i][0] = bp::extract<double>(t[0]);
				pa[i][1] = bp::extract<double>(t[1]);
				pa[i][2] = bp::extract<double>(t[2]);
			}
			m_data = pa;
			m_size = len;
			create(pa, prepareTempIndices(len), len);
			delete[] m_temp_indices;
		}

		size_t FindClosest(bp::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			size_t res = KDtreeIndexed::FindClosest(_pv, sqRad2, threadNum);
			delete[] _pv;
			return res;
		}

		bp::list fixedRangeSearch(bp::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<size_t> res = KDtreeIndexed::fixedRangeSearch(_pv, sqRad2, threadNum);
			bp::list l;
			for (auto &it: res) {
				l.append(it);
			}
			delete[] _pv;
			return l;
		}

		bp::list kNearestNeighbors(bp::tuple _p, size_t k)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<size_t> res = KDtreeIndexed::kNearestNeighbors(_pv, k, threadNum);
			bp::list l;
			for (auto &it: res) {
				l.append(it);
			}
			delete[] _pv;
			return l;
		}

		size_t segmentSearch_1NearestPoint(bp::tuple _p, bp::tuple _p0, double maxdist2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			double *_p0v = new double[3];
			_p0v[0] = bp::extract<double>(_p0[0]);
			_p0v[1] = bp::extract<double>(_p0[1]);
			_p0v[2] = bp::extract<double>(_p0[2]);
			int threadNum = 0;
			size_t res = KDtreeIndexed::segmentSearch_1NearestPoint(_pv, _p0v, maxdist2, threadNum);
			delete[] _pv;
			delete[] _p0v;
			return res;
		}

		~KDtreeIndexedWrapper()
		{
			for (size_t i = 0; i < m_size; ++i) {
				delete[] m_data[i];
			}
			delete[] m_data;
		}
};

// we need to wrap KDtree because its constructor takes a double**
// which boost python cannot handle directly
class KDtreeWrapper : public KDtree
{
	private:
		double **m_data;
		size_t m_size;

	public:
		KDtreeWrapper(bp::list l) : KDtree()
		{
			size_t len = bp::extract<std::size_t>(l.attr("__len__")());
			double** pa = new double*[len];
			for (size_t i = 0; i < len; ++i) {
				bp::tuple t = bp::extract<bp::tuple>(l[i]);
				pa[i] = new double[3];
				pa[i][0] = bp::extract<double>(t[0]);
				pa[i][1] = bp::extract<double>(t[1]);
				pa[i][2] = bp::extract<double>(t[2]);
			}
			m_data = pa;
			m_size = len;
			create(Void(), m_data, m_size);
		}

		bp::object FindClosest(bp::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			double *closest = KDtree::FindClosest(_pv, sqRad2, threadNum);
			delete[] _pv;
			if (closest == 0) {
				return bp::object(); // return None
			}
			bp::list l;
			for (int i = 0; i < 3; i++) {
				l.append(closest[i]);
			}
			return bp::tuple(l);
		}

		bp::list fixedRangeSearch(bp::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::fixedRangeSearch(_pv, sqRad2, threadNum);
			bp::list l;
			for (auto &it: res) {
				bp::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(bp::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		bp::list kNearestNeighbors(bp::tuple _p, size_t k)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::kNearestNeighbors(_pv, k, threadNum);
			bp::list l;
			for (auto &it: res) {
				bp::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(bp::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		bp::list kNearestRangeSearch(bp::tuple _p, size_t k, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::kNearestRangeSearch(_pv, k, sqRad2, threadNum);
			bp::list l;
			for (auto &it: res) {
				bp::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(bp::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		bp::object segmentSearch_1NearestPoint(bp::tuple _p, bp::tuple _p0, double maxdist2)
		{
			double *_pv = new double[3];
			_pv[0] = bp::extract<double>(_p[0]);
			_pv[1] = bp::extract<double>(_p[1]);
			_pv[2] = bp::extract<double>(_p[2]);
			double *_p0v = new double[3];
			_p0v[0] = bp::extract<double>(_p0[0]);
			_p0v[1] = bp::extract<double>(_p0[1]);
			_p0v[2] = bp::extract<double>(_p0[2]);
			int threadNum = 0;
			double *closest = KDtree::segmentSearch_1NearestPoint(_pv, _p0v, maxdist2, threadNum);
			delete[] _pv;
			delete[] _p0v;
			if (closest == 0) {
				return bp::object(); // return None
			}
			bp::list l;
			for (int i = 0; i < 3; i++) {
				l.append(closest[i]);
			}
			return bp::tuple(l);
		}

		~KDtreeWrapper()
		{
			for (size_t i = 0; i < m_size; ++i) {
				delete[] m_data[i];
			}
			delete[] m_data;
		}
};

void openDirectoryWrapper(
		bool scanserver,
		const std::string& directory,
		IOType type,
		int start,
		int end
		) {
	Scan::openDirectory(scanserver, directory, type, start, end);
}

bp::tuple calculateNormalWrapper(bp::list pts) {
	size_t num_pts = bp::extract<std::size_t>(pts.attr("__len__")());
	std::vector<Point> points;
	points.reserve(num_pts);
	for (size_t i = 0; i < num_pts; ++i) {
		bp::tuple t = bp::extract<bp::tuple>(pts[i]);
		points.push_back(Point(bp::extract<double>(t[0]), bp::extract<double>(t[1]), bp::extract<double>(t[2])));
	}
	double norm[3];
	double eigen[3];
	calculateNormal(points, norm, eigen);
	bp::list p1;
	p1.append(norm[0]);
	p1.append(norm[1]);
	p1.append(norm[2]);
	bp::list p2;
	p2.append(eigen[0]);
	p2.append(eigen[1]);
	p2.append(eigen[2]);
	bp::list p;
	p.append(bp::tuple(p1));
	p.append(bp::tuple(p2));
	return bp::tuple(p);
}

/*
 * pythons float.hex() always prints full precision
 *    (0.5).hex() => 0x1.0000000000000p-1
 * printf("%a") only prints as much as necessary
 *    printf("%a", 0.5) => 0x1p-1
 * python doesn't have %a
 */
std::string float2hex(double val)
{
	// -0x1.MMMMMMMMMMMMMp+EEE
	// 1 byte   optional sign
	// 4 bytes  0x1. prefix
	// 13 bytes mantissa
	// 2 bytes  p+ infix
	// 3 bytes  exponent
	// 1 byte   NULL
	// = 24 bytes maximum
	const int buflen = 24;
	char buffer[buflen];
	int ret = snprintf(buffer, buflen, "%a", val);
	if (ret < 0 || ret >= buflen) {
		PyErr_SetString(PyExc_RuntimeError, "snprintf failed");
		bp::throw_error_already_set();
	}
	return std::string(buffer);
}

void export_utils()
{
	bp::object utilsModule(bp::handle<>(bp::borrowed(PyImport_AddModule("py3dtk.utils"))));
	bp::scope().attr("utils") = utilsModule;
	bp::scope utils_scope = utilsModule; // set the scope to the new module
	bp::def("float2hex", float2hex);
}

BOOST_PYTHON_MODULE(py3dtk)
{
	bp::object package = bp::scope();
	package.attr("__path__") = "py3dtk";

	bp::enum_<IOType>("IOType")
		.value("UOS", UOS)
		.value("UOSR", UOSR)
		.value("UOS_MAP", UOS_MAP)
		.value("UOS_FRAMES", UOS_FRAMES)
		.value("UOS_MAP_FRAMES", UOS_MAP_FRAMES)
		.value("UOS_RGB", UOS_RGB)
		.value("UOS_RRGBT", UOS_RRGBT)
		.value("OLD", OLD)
		.value("RTS", RTS)
		.value("RTS_MAP", RTS_MAP)
		.value("RIEGL_TXT", RIEGL_TXT)
		.value("RIEGL_PROJECT", RIEGL_PROJECT)
		.value("RIEGL_RGB", RIEGL_RGB)
		.value("RIEGL_BIN", RIEGL_BIN)
		.value("IFP", IFP)
		.value("ZAHN", ZAHN)
		.value("PLY", PLY)
		.value("WRL", WRL)
		.value("XYZ", XYZ)
		.value("ZUF", ZUF)
		.value("ASC", ASC)
		.value("IAIS", IAIS)
		.value("FRONT", FRONT)
		.value("X3D", X3D)
		.value("RXP", RXP)
		.value("AIS", AIS)
		.value("OCT", OCT)
		.value("TXYZR", TXYZR)
		.value("XYZR", XYZR)
		.value("XYZ_RGB", XYZ_RGB)
		.value("KS", KS)
		.value("KS_RGB", KS_RGB)
		.value("STL", STL)
		.value("LAZ", LAZ)
		.value("LEICA", LEICA)
		.value("PCL", PCL)
		.value("PCI", PCI)
		.value("UOS_CAD", UOS_CAD)
		.value("VELODYNE", VELODYNE)
		.value("VELODYNE_FRAMES", VELODYNE_FRAMES)
		.value("UOS_RRGB", UOS_RRGB)
		.value("XYZ_RRGB", XYZ_RRGB)
		.value("FARO_XYZ_RGBR", FARO_XYZ_RGBR)
		.value("LEICA_XYZR", LEICA_XYZR)
        ;

	bp::enum_<Scan::AlgoType>("AlgoType")
		.value("INVALID", Scan::INVALID)
		.value("ICP", Scan::ICP)
		.value("ICPINACTIVE", Scan::ICPINACTIVE)
		.value("LUM", Scan::LUM)
		.value("ELCH", Scan::ELCH);

	bp::def("openDirectory", openDirectoryWrapper);

	bp::def("M4identity", pyM4identity);
	bp::def("transform3", pytransform3);
	bp::def("transform3normal", pytransform3normal);
	bp::def("EulerToMatrix3", pyEulerToMatrix3);
	bp::def("EulerToMatrix4", pyEulerToMatrix4);
	bp::def("M3inv", pyM3inv);
	bp::def("M4inv", pyM4inv);

	bp::def("formatname_to_io_type", formatname_to_io_type);
	bp::def("io_type_to_libname", io_type_to_libname);

	bp::class_<DataPointer>("DataPointer", bp::no_init);
	// DataXYZ is a TripleArray<double>
	bp::class_<DataXYZ, bp::bases<DataPointer>>("DataXYZ", bp::init<DataPointer&>())
		.def("__getitem__", &DataXYZ_getitem)
		.def("__len__", &DataXYZ_length);

	bp::class_<DataReflectance, bp::bases<DataPointer>>("DataReflectance", bp::init<DataPointer&>())
		.def("__getitem__", &DataReflectance_getitem)
		.def("__len__", &DataReflectance_length);

	// Scan is not copyable and has no init
	bp::class_<Scan, boost::noncopyable>("Scan", bp::no_init)
		.def("get", scan_getByString)
		.def("get", scan_getByType)
		.def("get_rPos", scan_get_rPos)
		.def("get_rPosTheta", scan_get_rPosTheta)
		.def("get_transMatOrg", scan_get_transMatOrg)
		.def("toGlobal", &Scan::toGlobal)
		.def("setRangeFilter", &Scan::setRangeFilter)
		.def("transform", scan_transform)
		.def("transformAll", scan_transformAll)
		.def("getIdentifier", &Scan::getIdentifier);
	// BasicScan derives from Scan, is not copyable and has no init
	bp::class_<BasicScan, boost::noncopyable, bp::bases<Scan>>("BasicScan", bp::no_init);

	// allows Python to hold the Scan* pointer in a variable
	bp::register_ptr_to_python<Scan*>();

	// create a type which wraps the std::vector<Scan*> in a class accessible
	// as a Python list
	bp::class_<std::vector<Scan*>>("ScanVector")
		.def(bp::vector_indexing_suite<std::vector<Scan*>>() );

	// add a global attribute as a pointer so that changes to it can be picked
	// up by Python
	bp::scope().attr("allScans") = bp::object(bp::ptr(&Scan::allScans));

	bp::class_<KDtreeIndexedWrapper>("KDtreeIndexed", bp::init<bp::list>())
		.def("FindClosest", &KDtreeIndexedWrapper::FindClosest)
		.def("fixedRangeSearch", &KDtreeIndexedWrapper::fixedRangeSearch)
		.def("kNearestNeighbors", &KDtreeIndexedWrapper::kNearestNeighbors)
		.def("segmentSearch_1NearestPoint", &KDtreeIndexedWrapper::segmentSearch_1NearestPoint);

	bp::class_<KDtreeWrapper>("KDtree", bp::init<bp::list>())
		.def("FindClosest", &KDtreeWrapper::FindClosest)
		.def("fixedRangeSearch", &KDtreeWrapper::fixedRangeSearch)
		.def("kNearestNeighbors", &KDtreeWrapper::kNearestNeighbors)
		.def("kNearestRangeSearch", &KDtreeWrapper::kNearestRangeSearch)
		.def("segmentSearch_1NearestPoint", &KDtreeWrapper::segmentSearch_1NearestPoint);

	bp::def("calculateNormal", calculateNormalWrapper);

	bp::class_<QuadTree, boost::noncopyable>("QuadTree", bp::init<DataXYZ const&>())
		.def("search", &QuadTree_search);

	export_utils();
}
