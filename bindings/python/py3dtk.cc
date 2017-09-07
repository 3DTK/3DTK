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

using namespace boost::python;

DataPointer (Scan::*scan_getByString)(const std::string&) = &Scan::get;
void (Scan::*scan_getByType)(IODataType) = &Scan::get;

boost::python::list QuadTree_search(QuadTree &tree, boost::python::tuple _p, double r)
{
	double *_pv = new double[3];
	_pv[0] = extract<double>(_p[0]);
	_pv[1] = extract<double>(_p[1]);
	_pv[2] = extract<double>(_p[2]);
	std::vector<size_t> res = tree.search(_pv, r);
	delete[] _pv;
	boost::python::list l;
	for (auto &it: res) {
		l.append(it);
	}
	return l;
}

boost::python::tuple scan_get_rPos(Scan &s)
{
	const double *rPos = s.get_rPos();
	boost::python::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(rPos[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple scan_get_rPosTheta(Scan &s)
{
	const double *rPosTheta = s.get_rPosTheta();
	boost::python::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(rPosTheta[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple scan_get_transMatOrg(Scan &s)
{
	const double *transMatOrg = s.get_transMatOrg();
	boost::python::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(transMatOrg[i]);
	}
	return boost::python::tuple(l);
}

void scan_transform(Scan &s, boost::python::tuple m, const Scan::AlgoType type, bool islum = 0)
{
	const double matrix[16] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
		boost::python::extract<double>(m[9])(),
		boost::python::extract<double>(m[10])(),
		boost::python::extract<double>(m[11])(),
		boost::python::extract<double>(m[12])(),
		boost::python::extract<double>(m[13])(),
		boost::python::extract<double>(m[14])(),
		boost::python::extract<double>(m[15])()
	};
	s.transform(matrix, type, islum);
}

void scan_transformAll(Scan &s, boost::python::tuple m)
{
	const double matrix[16] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
		boost::python::extract<double>(m[9])(),
		boost::python::extract<double>(m[10])(),
		boost::python::extract<double>(m[11])(),
		boost::python::extract<double>(m[12])(),
		boost::python::extract<double>(m[13])(),
		boost::python::extract<double>(m[14])(),
		boost::python::extract<double>(m[15])()
	};
	s.transformAll(matrix);
}

boost::python::tuple pyM4identity()
{
	boost::python::list l;
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
	return boost::python::tuple(l);
}

boost::python::tuple pytransform3(boost::python::tuple m, boost::python::tuple p)
{
	const double matrix[16] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
		boost::python::extract<double>(m[9])(),
		boost::python::extract<double>(m[10])(),
		boost::python::extract<double>(m[11])(),
		boost::python::extract<double>(m[12])(),
		boost::python::extract<double>(m[13])(),
		boost::python::extract<double>(m[14])(),
		boost::python::extract<double>(m[15])()
	};
	double point[3] = {
		boost::python::extract<double>(p[0])(),
		boost::python::extract<double>(p[1])(),
		boost::python::extract<double>(p[2])(),
	};
	transform3(matrix, point);
	boost::python::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(point[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple pytransform3normal(boost::python::tuple m, boost::python::tuple n)
{
	const double matrix[16] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
		boost::python::extract<double>(m[9])(),
		boost::python::extract<double>(m[10])(),
		boost::python::extract<double>(m[11])(),
		boost::python::extract<double>(m[12])(),
		boost::python::extract<double>(m[13])(),
		boost::python::extract<double>(m[14])(),
		boost::python::extract<double>(m[15])()
	};
	double normal[3] = {
		boost::python::extract<double>(n[0])(),
		boost::python::extract<double>(n[1])(),
		boost::python::extract<double>(n[2])(),
	};
	transform3normal(matrix, normal);
	boost::python::list l;
	for (int i = 0; i < 3; ++i) {
		l.append(normal[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple pyEulerToMatrix3(boost::python::tuple t)
{
	const double rPosTheta[3] = {
		boost::python::extract<double>(t[0])(),
		boost::python::extract<double>(t[1])(),
		boost::python::extract<double>(t[2])(),
	};
	double matrix[9];
	EulerToMatrix3(rPosTheta, matrix);
	boost::python::list l;
	for (int i = 0; i < 9; ++i) {
		l.append(matrix[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple pyEulerToMatrix4(boost::python::tuple p, boost::python::tuple t)
{
	const double rPos[3] = {
		boost::python::extract<double>(p[0])(),
		boost::python::extract<double>(p[1])(),
		boost::python::extract<double>(p[2])(),
	};
	const double rPosTheta[3] = {
		boost::python::extract<double>(t[0])(),
		boost::python::extract<double>(t[1])(),
		boost::python::extract<double>(t[2])(),
	};
	double matrix[16];
	EulerToMatrix4(rPos, rPosTheta, matrix);
	boost::python::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(matrix[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple pyM3inv(boost::python::tuple m)
{
	const double Min[9] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
	};
	double Mout[9];
	M3inv(Min, Mout);
	boost::python::list l;
	for (int i = 0; i < 9; ++i) {
		l.append(Mout[i]);
	}
	return boost::python::tuple(l);
}

boost::python::tuple pyM4inv(boost::python::tuple m)
{
	const double Min[16] = {
		boost::python::extract<double>(m[0])(),
		boost::python::extract<double>(m[1])(),
		boost::python::extract<double>(m[2])(),
		boost::python::extract<double>(m[3])(),
		boost::python::extract<double>(m[4])(),
		boost::python::extract<double>(m[5])(),
		boost::python::extract<double>(m[6])(),
		boost::python::extract<double>(m[7])(),
		boost::python::extract<double>(m[8])(),
		boost::python::extract<double>(m[9])(),
		boost::python::extract<double>(m[10])(),
		boost::python::extract<double>(m[11])(),
		boost::python::extract<double>(m[12])(),
		boost::python::extract<double>(m[13])(),
		boost::python::extract<double>(m[14])(),
		boost::python::extract<double>(m[15])()
	};
	double Mout[16];
	M4inv(Min, Mout);
	boost::python::list l;
	for (int i = 0; i < 16; ++i) {
		l.append(Mout[i]);
	}
	return boost::python::tuple(l);
}

// given a DataXYZ and an index, assemble a Python tuple to return
// that contains the xyz data
boost::python::tuple DataXYZ_getitem(DataXYZ &s, size_t index)
{
	if (index >= 0 && index < s.size()) {
		boost::python::list l;
		for (int i = 0; i < 3; ++i) {
			l.append(s[index][i]);
		}
		return boost::python::tuple(l);
	} else {
		PyErr_SetString(PyExc_IndexError, "index out of range");
		throw_error_already_set();
		// because of the thrown exception, this will never be reached
		// we do this to make the compiler happy
		return boost::python::tuple();
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
		throw_error_already_set();
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
		KDtreeIndexedWrapper(boost::python::list l) : KDtreeIndexed()
		{
			size_t len = extract<std::size_t>(l.attr("__len__")());
			double** pa = new double*[len];
			for (size_t i = 0; i < len; ++i) {
				boost::python::tuple t = extract<boost::python::tuple>(l[i]);
				pa[i] = new double[3];
				pa[i][0] = extract<double>(t[0]);
				pa[i][1] = extract<double>(t[1]);
				pa[i][2] = extract<double>(t[2]);
			}
			m_data = pa;
			m_size = len;
			create(pa, prepareTempIndices(len), len);
			delete[] m_temp_indices;
		}

		size_t FindClosest(boost::python::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			size_t res = KDtreeIndexed::FindClosest(_pv, sqRad2, threadNum);
			delete[] _pv;
			return res;
		}

		boost::python::list fixedRangeSearch(boost::python::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<size_t> res = KDtreeIndexed::fixedRangeSearch(_pv, sqRad2, threadNum);
			boost::python::list l;
			for (auto &it: res) {
				l.append(it);
			}
			delete[] _pv;
			return l;
		}

		boost::python::list kNearestNeighbors(boost::python::tuple _p, size_t k)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<size_t> res = KDtreeIndexed::kNearestNeighbors(_pv, k, threadNum);
			boost::python::list l;
			for (auto &it: res) {
				l.append(it);
			}
			delete[] _pv;
			return l;
		}

		size_t segmentSearch_1NearestPoint(boost::python::tuple _p, boost::python::tuple _p0, double maxdist2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			double *_p0v = new double[3];
			_p0v[0] = extract<double>(_p0[0]);
			_p0v[1] = extract<double>(_p0[1]);
			_p0v[2] = extract<double>(_p0[2]);
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
		KDtreeWrapper(boost::python::list l) : KDtree()
		{
			size_t len = extract<std::size_t>(l.attr("__len__")());
			double** pa = new double*[len];
			for (size_t i = 0; i < len; ++i) {
				boost::python::tuple t = extract<boost::python::tuple>(l[i]);
				pa[i] = new double[3];
				pa[i][0] = extract<double>(t[0]);
				pa[i][1] = extract<double>(t[1]);
				pa[i][2] = extract<double>(t[2]);
			}
			m_data = pa;
			m_size = len;
			create(Void(), m_data, m_size);
		}

		boost::python::object FindClosest(boost::python::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			double *closest = KDtree::FindClosest(_pv, sqRad2, threadNum);
			delete[] _pv;
			if (closest == 0) {
				return boost::python::object(); // return None
			}
			boost::python::list l;
			for (int i = 0; i < 3; i++) {
				l.append(closest[i]);
			}
			return boost::python::tuple(l);
		}

		boost::python::list fixedRangeSearch(boost::python::tuple _p, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::fixedRangeSearch(_pv, sqRad2, threadNum);
			boost::python::list l;
			for (auto &it: res) {
				boost::python::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(boost::python::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		boost::python::list kNearestNeighbors(boost::python::tuple _p, size_t k)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::kNearestNeighbors(_pv, k, threadNum);
			boost::python::list l;
			for (auto &it: res) {
				boost::python::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(boost::python::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		boost::python::list kNearestRangeSearch(boost::python::tuple _p, size_t k, double sqRad2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			int threadNum = 0;
			std::vector<Point> res = KDtree::kNearestRangeSearch(_pv, k, sqRad2, threadNum);
			boost::python::list l;
			for (auto &it: res) {
				boost::python::list p;
				p.append(it.x);
				p.append(it.y);
				p.append(it.z);
				l.append(boost::python::tuple(p));
			}
			delete[] _pv;
			return l;
		}

		boost::python::object segmentSearch_1NearestPoint(boost::python::tuple _p, boost::python::tuple _p0, double maxdist2)
		{
			double *_pv = new double[3];
			_pv[0] = extract<double>(_p[0]);
			_pv[1] = extract<double>(_p[1]);
			_pv[2] = extract<double>(_p[2]);
			double *_p0v = new double[3];
			_p0v[0] = extract<double>(_p0[0]);
			_p0v[1] = extract<double>(_p0[1]);
			_p0v[2] = extract<double>(_p0[2]);
			int threadNum = 0;
			double *closest = KDtree::segmentSearch_1NearestPoint(_pv, _p0v, maxdist2, threadNum);
			delete[] _pv;
			delete[] _p0v;
			if (closest == 0) {
				return boost::python::object(); // return None
			}
			boost::python::list l;
			for (int i = 0; i < 3; i++) {
				l.append(closest[i]);
			}
			return boost::python::tuple(l);
		}

		~KDtreeWrapper()
		{
			for (size_t i = 0; i < m_size; ++i) {
				delete[] m_data[i];
			}
			delete[] m_data;
		}
};


boost::python::tuple calculateNormalWrapper(boost::python::list pts) {
	size_t num_pts = extract<std::size_t>(pts.attr("__len__")());
	std::vector<Point> points;
	points.reserve(num_pts);
	for (size_t i = 0; i < num_pts; ++i) {
		boost::python::tuple t = extract<boost::python::tuple>(pts[i]);
		points.push_back(Point(extract<double>(t[0]), extract<double>(t[1]), extract<double>(t[2])));
	}
	double norm[3];
	double eigen[3];
	calculateNormal(points, norm, eigen);
	boost::python::list p1;
	p1.append(norm[0]);
	p1.append(norm[1]);
	p1.append(norm[2]);
	boost::python::list p2;
	p2.append(eigen[0]);
	p2.append(eigen[1]);
	p2.append(eigen[2]);
	boost::python::list p;
	p.append(boost::python::tuple(p1));
	p.append(boost::python::tuple(p2));
	return boost::python::tuple(p);
}


BOOST_PYTHON_MODULE(py3dtk)
{
	enum_<IOType>("IOType")
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
		.value("B3D", B3D)
        ;

	enum_<Scan::AlgoType>("AlgoType")
		.value("INVALID", Scan::INVALID)
		.value("ICP", Scan::ICP)
		.value("ICPINACTIVE", Scan::ICPINACTIVE)
		.value("LUM", Scan::LUM)
		.value("ELCH", Scan::ELCH);

	def("openDirectory", Scan::openDirectory);

	def("M4identity", pyM4identity);
	def("transform3", pytransform3);
	def("transform3normal", pytransform3normal);
	def("EulerToMatrix3", pyEulerToMatrix3);
	def("EulerToMatrix4", pyEulerToMatrix4);
	def("M3inv", pyM3inv);
	def("M4inv", pyM4inv);

	def("formatname_to_io_type", formatname_to_io_type);
	def("io_type_to_libname", io_type_to_libname);

	class_<DataPointer>("DataPointer", boost::python::no_init);
	// DataXYZ is a TripleArray<double>
	class_<DataXYZ, boost::python::bases<DataPointer>>("DataXYZ", boost::python::init<DataPointer&>())
		.def("__getitem__", &DataXYZ_getitem)
		.def("__len__", &DataXYZ_length);

	class_<DataReflectance, boost::python::bases<DataPointer>>("DataReflectance", boost::python::init<DataPointer&>())
		.def("__getitem__", &DataReflectance_getitem)
		.def("__len__", &DataReflectance_length);

	// Scan is not copyable and has no init
	class_<Scan, boost::noncopyable>("Scan", boost::python::no_init)
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
	class_<BasicScan, boost::noncopyable, boost::python::bases<Scan>>("BasicScan", boost::python::no_init);

	// allows Python to hold the Scan* pointer in a variable
	register_ptr_to_python<Scan*>();

	// create a type which wraps the std::vector<Scan*> in a class accessible
	// as a Python list
	class_<std::vector<Scan*>>("ScanVector")
		.def(vector_indexing_suite<std::vector<Scan*>>() );

	// add a global attribute as a pointer so that changes to it can be picked
	// up by Python
	scope().attr("allScans") = object(ptr(&Scan::allScans));

	class_<KDtreeIndexedWrapper>("KDtreeIndexed", boost::python::init<boost::python::list>())
		.def("FindClosest", &KDtreeIndexedWrapper::FindClosest)
		.def("fixedRangeSearch", &KDtreeIndexedWrapper::fixedRangeSearch)
		.def("kNearestNeighbors", &KDtreeIndexedWrapper::kNearestNeighbors)
		.def("segmentSearch_1NearestPoint", &KDtreeIndexedWrapper::segmentSearch_1NearestPoint);

	class_<KDtreeWrapper>("KDtree", boost::python::init<boost::python::list>())
		.def("FindClosest", &KDtreeWrapper::FindClosest)
		.def("fixedRangeSearch", &KDtreeWrapper::fixedRangeSearch)
		.def("kNearestNeighbors", &KDtreeWrapper::kNearestNeighbors)
		.def("kNearestRangeSearch", &KDtreeWrapper::kNearestRangeSearch)
		.def("segmentSearch_1NearestPoint", &KDtreeWrapper::segmentSearch_1NearestPoint);

	def("calculateNormal", calculateNormalWrapper);

	class_<QuadTree, boost::noncopyable>("QuadTree", boost::python::init<DataXYZ const&>())
		.def("search", &QuadTree_search);
}
