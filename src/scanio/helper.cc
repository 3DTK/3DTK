#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <climits>
#include "scanio/helper.h"
#include "slam6d/globals.icc"
#ifdef WITH_LIBZIP
#include <zip.h>
#endif

bool ScanDataTransform_identity::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    return true;
}

bool ScanDataTransform_ks::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    double tmp;

    // the enemy's x/y/z is mapped to slam's x/z/y, shuffle time!
    tmp = xyz[1];
    xyz[1] = xyz[2];
    xyz[2] = tmp;

    // TODO: offset is application specific, handle with care
    // correct constant offset (in slam coordinates)
    xyz[0] -= 70000.0; // x
    xyz[2] -= 20000.0; // z

    // convert coordinate to cm
    xyz[0] *= 100.0;
    xyz[1] *= 100.0;
    xyz[2] *= 100.0;

    return true;
}

bool ScanDataTransform_riegl::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    double tmp;
    tmp = xyz[2];
    xyz[2] = 100.0 * xyz[0];
    xyz[0] = -100.0 * xyz[1];
    xyz[1] = 100.0 * tmp;

    return true;
}

bool ScanDataTransform_rts::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    // check if TYPE_INVALID flag for rts invalid points is set
    if(*type & 0x10)
        return false;

    double tmp;
    tmp = xyz[2];
    xyz[2] = 0.1 * xyz[0];
    xyz[0] = 0.1 * xyz[1];
    xyz[1] = -0.1 * tmp;

    return true;
}

bool ScanDataTransform_xyz::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    double tmp;
    tmp = xyz[2];
    xyz[2] = 100.0*xyz[0];
    xyz[0] = -100.0*xyz[1];
    xyz[1] = 100.0*tmp;

    return true;
}

bool ScanDataTransform_pts::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
    xyz[0] = xyz[0];
    xyz[1] = xyz[1];
    xyz[2] = -1 * xyz[2];

    return true;
}

bool ScanDataTransform_combined::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
  return m_sdt_1.transform(xyz, rgb, refl, temp, ampl, type, devi, n)
    && m_sdt_2.transform(xyz, rgb, refl, temp, ampl, type, devi, n);
}

bool ScanDataTransform_matrix::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3])
{
  transform3(m_matrix, xyz);
  return true;
}

time_t lastModifiedHelper(const char *dir_path,
        const char *identifier,
        const char **data_path_suffixes,
        const char *data_path_prefix,
        unsigned int id_len)
{
    for (const char **s = data_path_suffixes; *s != 0; s++) {
        boost::filesystem::path data(dir_path);
        data /= boost::filesystem::path(std::string(data_path_prefix) + identifier + *s);
        if (boost::filesystem::exists(data)) {
            return boost::filesystem::last_write_time(data);
        }
    }
    throw std::runtime_error(std::string("Data file could not be opened for [") +
            identifier + "] in [" + dir_path + "]");
}

std::list<std::string> readDirectoryHelper(const char *dir_path,
        unsigned int start,
        unsigned int end,
        const char **data_path_suffixes,
        const char *data_path_prefix,
        unsigned int id_len)
{
    std::list<std::string> identifiers;
    for (unsigned int i = start; i <= end; ++i) {
        // identifier is /d/d/d (000-999)
        std::string identifier(to_string(i, id_len));
        // scan consists of data and pose files
        bool found = false;
        for (const char **s = data_path_suffixes; *s != 0; s++) {
            boost::filesystem::path data(dir_path);
            data /= boost::filesystem::path(std::string(data_path_prefix) + identifier + *s);
            PointFilter filter;
            /* pass the identity function because we don't want to read data
             * from the file but just find out whether it exists or not */
            if (open_path(data, [](std::istream &data_file) -> bool { return true; })) {
                found = true;
                break;
            }
        }
        // stop if part of a scan is missing or end by absence is detected
        if (!found) {
            std::cerr << "No data found for " << data_path_prefix << identifier << "!" << std::endl;
            break;
        }
        identifiers.push_back(identifier);
    }
    return identifiers;
}

std::list<std::string> readDirectoryHelper(dataset_settings& dss,
  const char **data_path_suffixes,
  const char *data_path_prefix,
  unsigned int id_len)
{
  //return readDirectoryHelper(dss.data_source.c_str(), dss.scan_numbers.min, dss.scan_numbers.max, data_path_suffixes, data_path_prefix);
  std::list<std::string> identifiers;
  multi_range_set::iterator it = dss.scan_ranges.begin();
  bool found = false;
  int last_id = *it;
  do {
    for (; !it.clusterDone(); ++it) {
      std::string identifier(to_string(*it, id_len));
      // scan consists of data and pose files
      found = false;
      for (const char **s = data_path_suffixes; *s != 0; s++) {
        boost::filesystem::path data(dss.data_source);
        data /= boost::filesystem::path(std::string(data_path_prefix) + identifier + *s);
        PointFilter filter;
        /* pass the identity function because we don't want to read data
         * from the file but just find out whether it exists or not */
        if (open_path(data, [](std::istream &data_file) -> bool { return true; })) {
          found = true;
          break;
        }
      }
      // stop if part of a scan is missing or end by absence is detected
      if (!found) {
        std::cerr << "No data found for " << data_path_prefix << identifier << "!" << std::endl;
        it.reference()->setMaxLimit(last_id);
        break;
      }
      else {
        last_id = *it;
      }
    }
    multi_range_set cluster = it.cluster();
    if (cluster.isValid())
    {
      identifiers.push_back(cluster.toString(id_len));
    }
    it.nextCluster();
  } while (!it.done());
  return identifiers;
}

void readPoseHelper(const char *dir_path,
        const char *identifier,
        double *pose,
        const char *pose_path_suffix,
        const char *pose_path_prefix)
{
    std::string id_str(identifier);
    multi_range<range<int> > mr;
    mr.set(id_str);
    mr.merged = true;
    id_str = to_string(*(mr.begin()), 3).c_str();

    boost::filesystem::path pose_path(dir_path);
    pose_path /= boost::filesystem::path(std::string(pose_path_prefix) + id_str +
            pose_path_suffix);


    /* the handler function passed to open_path() will fill the pose[] array
     * */
    bool res = open_path(pose_path, [=](std::istream &data_file) -> bool {
                //get pose from .frames files
                if(strcmp(pose_path_suffix, ".frames") == 0) {
                  double tMatrix[17];
                  double rPos[3], rPosTheta[3];
                  std::string buffer, line;
                  while(getline(data_file, buffer)) line = buffer;
                  std::istringstream iss(line);
                  for (unsigned int i = 0; i < 17; iss >> tMatrix[i++]);
                  Matrix4ToEuler(tMatrix, rPosTheta, rPos);
                  for (unsigned int i = 0; i < 3; i++) {
                    pose[i] = rPos[i];
                    pose[i+3] = rPosTheta[i];
                  }
                  return true;
                //read pose from .pose files
                } else {
                  // read 6 plain doubles
                  for (int i = 0; i < 6; ++i) data_file >> pose[i];
                  // convert angles from deg to rad
                  for (int i = 3; i < 6; ++i) pose[i] = rad(pose[i]);
                  return true;
                }
            });
    if (!res)
        throw std::runtime_error(std::
                string
                ("Pose file could not be opened for [") +
                identifier + "] in [" + dir_path + "]");
}

bool strtoval(char *pos, unsigned int linenr, double* ret)
{
    char *endptr;
    errno = 0;
    char *saved_locale;
    saved_locale = setlocale(LC_NUMERIC, "C");
    double val = strtod(pos, &endptr);
    setlocale(LC_NUMERIC, saved_locale);

    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << std::endl;
        if (val == HUGE_VAL) {
            std::cerr << "overflow" << std::endl;
        } else if (val == 0) {
            std::cerr << "underflow" << std::endl;
        }
        perror("strod");
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << std::endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << std::endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, float* ret)
{
    char *endptr;
    errno = 0;
    char *saved_locale;
    saved_locale = setlocale(LC_NUMERIC, "C");
    float val = strtof(pos, &endptr);
    setlocale(LC_NUMERIC, saved_locale);

    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << std::endl;
        if (val == HUGE_VALF) {
            std::cerr << "overflow" << std::endl;
        } else if (val == 0) {
            std::cerr << "underflow" << std::endl;
        }
        perror("strof");
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << std::endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << std::endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, unsigned char* ret)
{
    char *endptr;
    errno = 0;
    char *saved_locale;
    saved_locale = setlocale(LC_NUMERIC, "C");
    long val = strtol(pos, &endptr, 10);
    setlocale(LC_NUMERIC, saved_locale);

    if (errno != 0 && val == 0) {
        std::cerr << "error in line " << linenr << std::endl;
        perror("strol");
        return false;
    }
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << std::endl;
        if (val < 0)
            std::cerr << "cannot be smaller than 0" << std::endl;
        if (val > 255)
            std::cerr << "cannot be greater than 255" << std::endl;
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << std::endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << std::endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, int* ret)
{
    char *endptr;
    errno = 0;
    char *saved_locale;
    saved_locale = setlocale(LC_NUMERIC, "C");
    long val = strtol(pos, &endptr, 10);
    setlocale(LC_NUMERIC, saved_locale);
    if (errno != 0 && val == 0) {
        std::cerr << "error in line " << linenr << std::endl;
        perror("strol");
        return false;
    }
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << std::endl;
        if (val < INT_MIN)
            std::cerr << "cannot be smaller than " << INT_MIN << std::endl;
        if (val > INT_MAX)
            std::cerr << "cannot be greater than " << INT_MAX << std::endl;
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << std::endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << std::endl;
        return false;
    }
    *ret = val;
    return true;
}

unsigned int strtoarray(std::string opt_s, char **&opts_array,const char * separator)
{
    char * opts = (char *)malloc(opt_s.size() + 1);
    memcpy(opts, opt_s.c_str(), opt_s.size() + 1);
    char * tmp = (char *)malloc(opt_s.size() + 1);
    memcpy(tmp, opt_s.c_str(), opt_s.size() + 1);

    char * pch;
    pch = strtok (opts,separator);

    int count = 0;
    while (pch != NULL)
    {
      count++;
      pch = strtok (NULL, separator);
    }
    opts_array = new char*[count + 1];
    pch = strtok (tmp,separator);
    int cnt = 0;
    while (pch != NULL && cnt < count)
    {
      cnt++;
      opts_array[cnt] = new char[strlen(pch) + 1];
      memcpy(opts_array[cnt], pch, strlen(pch) + 1);
      pch = strtok (NULL, separator);
    }
    return count;
}

bool storeval(char *pos, unsigned int linenr, IODataType currspec, double* xyz, int* xyz_idx, unsigned char* rgb, int* rgb_idx, float* refl, float* temp, float* ampl, int* type, float* devi, double* n, int* n_idx)
{
    switch (currspec) {
        case DATA_XYZ:
            return strtoval(pos, linenr, &xyz[(*xyz_idx)++]);
        case DATA_RGB:
            return strtoval(pos, linenr, &rgb[(*rgb_idx)++]);
        case DATA_REFLECTANCE:
            return strtoval(pos, linenr, refl);
        case DATA_TEMPERATURE:
            return strtoval(pos, linenr, temp);
        case DATA_AMPLITUDE:
            return strtoval(pos, linenr, ampl);
        case DATA_TYPE:
            return strtoval(pos, linenr, type);
        case DATA_DEVIATION:
            return strtoval(pos, linenr, devi);
        case DATA_NORMAL:
            return strtoval(pos, linenr, &n[(*n_idx)++]);
        case DATA_DUMMY:
            return true;
        case DATA_TERMINATOR:
            std::cerr << "too many values in line " << linenr << std::endl;
            return false;
        default:
            std::cerr << "storeval failed at " << linenr << std::endl;
            return false;
    }
}

bool checkSpec(IODataType* spec, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* refl, std::vector<float>* temp, std::vector<float>* ampl, std::vector<int>* type, std::vector<float>* devi, std::vector<double>* n)
{
    int count = 0;
    int xyzcount = 0;
    int rgbcount = 0;
    int reflcount = 0;
    int tempcount = 0;
    int amplcount = 0;
    int typecount = 0;
    int devicount = 0;
    int normalcount = 0;
    IODataType *currspec;
    // go through the spec array and count the occurrence of each spec
    for (currspec = spec; *currspec != DATA_TERMINATOR; ++currspec) {
        switch (*currspec) {
            case DATA_XYZ:
                xyzcount++;
                count++;
                break;
            case DATA_RGB:
                rgbcount++;
                count++;
                break;
            case DATA_REFLECTANCE:
                reflcount++;
                count++;
                break;
            case DATA_TEMPERATURE:
                tempcount++;
                count++;
                break;
            case DATA_AMPLITUDE:
                amplcount++;
                count++;
                break;
            case DATA_TYPE:
                typecount++;
                count++;
                break;
            case DATA_DEVIATION:
                devicount++;
                count++;
                break;
            case DATA_NORMAL:
                normalcount++;
                break;
            case DATA_DUMMY:
                break;
            default:
                std::cerr << "unknown spec: " << *currspec;
                return false;
        }
    }
    if (count == 0) {
        std::cerr << "must supply more than zero specs" << std::endl;
        return false;
    }
    // check if the spec matches the supplied vectors
    if (xyz == 0 && xyzcount != 0) {
        std::cerr << "you gave a xyz spec but no xyz vector" << std::endl;
        return false;
    }
    if (xyz != 0 && xyzcount != 3) {
        std::cerr << "you gave a xyz vector, so you must supply exactly three xyz specs" << std::endl;
        return false;
    }
    if (rgb == 0 && rgbcount != 0) {
        std::cerr << "you gave a rgb spec but no rgb vector" << std::endl;
        return false;
    }
    if (rgb != 0 && rgbcount != 3) {
        std::cerr << "you gave a rgb vector, so you must supply exactly three rgb specs" << std::endl;
        return false;
    }
    if (refl == 0 && reflcount != 0) {
        std::cerr << "you gave a reflection spec but no reflection vector" << std::endl;
        return false;
    }
    if (refl != 0 && reflcount != 1) {
        std::cerr << "you gave a reflection vector, so you must supply exactly one reflection spec" << std::endl;
        return false;
    }
    if (temp == 0 && tempcount != 0) {
        std::cerr << "you gave a temperature spec but no temperature vector" << std::endl;
        return false;
    }
    if (temp != 0 && tempcount != 1) {
        std::cerr << "you gave a temperature vector, so you must supply exactly one temperature spec" << std::endl;
        return false;
    }
    if (ampl == 0 && amplcount != 0) {
        std::cerr << "you gave an amplitude spec but no amplitude vector" << std::endl;
        return false;
    }
    if (ampl != 0 && amplcount != 1) {
        std::cerr << "you gave an amplitude vector, so you must supply exactly one amplitude spec" << std::endl;
        return false;
    }
    if (type == 0 && typecount != 0) {
        std::cerr << "you gave a type spec but no type vector" << std::endl;
        return false;
    }
    if (type != 0 && typecount != 1) {
        std::cerr << "you gave a type vector, so you must supply exactly one type spec" << std::endl;
        return false;
    }
    if (devi == 0 && devicount != 0) {
        std::cerr << "you gave a deviation spec but no deviation vector" << std::endl;
        return false;
    }
    if (devi != 0 && devicount != 1) {
        std::cerr << "you gave a deviation vector, so you must supply exactly one deviation spec" << std::endl;
        return false;
    }
    if (n == 0 && normalcount != 0) {
        std::cerr << "you gave a normal spec but no normal vector" << std::endl;
        return false;
    }
    if (n != 0 && normalcount != 3) {
        std::cerr << "you gave a normal vector, so you must supply exactly three normal specs" << std::endl;
        return false;
    }
    return true;
}

/* this is a wrapper around readASCII which should work for
 * most of the text based input formats like uos* and xyz*
 *
 * This function returns another function so that it can be passed to the
 * open_path() function. This wrapping is necessary because the open_path()
 * function only accepts a function taking the istream as an argument but
 * the result of reading the istream has to be stored somewhere. Passing the
 * pointers and references around correctly is part of this lambda wrapper.
 * */
std::function<bool (std::istream &data_file)> open_uos_file(
        IODataType* spec, ScanDataTransform& transform, PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation, std::vector<double>* normal)
{
    return [=,&filter,&transform](std::istream &data_file) -> bool {
    return readASCII(data_file, spec, transform, filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation, normal);
    };
}


/* used by readASCII to read a single line
 *
 * splitting this function out of readASCII became necessary to facilitate the
 * check against the optional first line */
bool handle_line(char *pos, std::streamsize linelen, unsigned int linenr, IODataType *currspec,
ScanDataTransform& transform, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned
        char>* rgb, std::vector<float>* refl, std::vector<float>* temp,
        std::vector<float>* ampl, std::vector<int>* type, std::vector<float>*
        devi, std::vector<double>* n)
{
    // temporary storage areas
    double xyz_tmp[3];
    unsigned char rgb_tmp[3];
    double n_tmp[3];
    float refl_tmp, temp_tmp, ampl_tmp, devi_tmp;
    int type_tmp;
    int xyz_idx = 0;
    int rgb_idx = 0;
    int n_idx   = 0;

    // skip over leading whitespace
    for (; isblank(*pos); ++pos, --linelen);
    // skip this line if it is empty
    if (linelen == 0) {
        return true;
    }
    // skip the line if it starts with the comment character
    if (*pos == '#')
        return true;

    char *cur;
    // now go through all fields and handle them according to the spec
    for (cur = pos; *cur != '\0' && *cur != '#'; ++cur) {
        // skip over everything that is not part of a field
        if (!isblank(*cur))
            continue;
        // we found the end of a field so lets read its content
        *cur = '\0';
        if (!storeval(pos, linenr, *currspec, xyz_tmp, &xyz_idx, rgb_tmp,
                    &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp, n_tmp, &n_idx))
            return false;
        currspec++;
        // read in the remaining whitespace
        pos = cur + 1;
        for (; isblank(*pos); ++pos);
        cur = pos - 1;
    }
    // read in last value (if any)
    if (*pos != '#' && *pos != '\0') {
        *cur = '\0';
        // read in the last value
        if (!storeval(pos, linenr, *currspec, xyz_tmp, &xyz_idx, rgb_tmp,
                    &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp, n_tmp, &n_idx))
            return false;
        // check if more values were expected
        currspec++;
    }
    if (*currspec != DATA_TERMINATOR) {
        std::cerr << "less values than in spec in line " << linenr << std::endl;
        return false;
    }
    // check if three values were read in for xyz and rgb
    if (xyz != 0 && xyz_idx != 3) {
        std::cerr << "can't understand " << xyz_idx << " coordinate values in line " << linenr << std::endl;
        return false;
    }
    if (rgb != 0 && rgb_idx != 3) {
        std::cerr << "can't understand " << rgb_idx << " color values in line " << linenr << std::endl;
        return false;
    }
    if (n != 0 && n_idx != 3) {
        std::cerr << "can't understand " << n_idx << " normal values in line " << linenr << std::endl;
        return false;
    }
    // apply transformations and filter data and append to vectors
    //
    // We catch out-of-memory exceptions which might easily happen because
    // double the actual amount of memory is needed in the worst case. This is
    // because the vector container will double the reserved memory when the
    // currently reserved size becomes full.
    //
    // FIXME: use a datastructure that allocates memory more conservatively
    //
    // FIXME: instead of using a different datastructure, another idea would
    //        be to use mmap-ed file(s) with the respective data inside
    if (transform.transform(xyz_tmp, rgb_tmp, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp, n_tmp)
            && (xyz == 0 || filter.check(xyz_tmp)) ) {
            if (xyz != 0)
                for (int i = 0; i < 3; ++i) {
                    try {
                        xyz->push_back(xyz_tmp[i]);
                    } catch (std::bad_alloc& ba){
                        std::cerr << "handle_line: Cannot add element to xyz vector with " << xyz->size() << " elements." << std::endl;
                        throw;
                    }
                }
            if (rgb != 0)
                for (int i = 0; i < 3; ++i) {
                    try {
                        rgb->push_back(rgb_tmp[i]);
                    } catch (std::bad_alloc& ba){
                        std::cerr << "handle_line: Cannot add element to rgb vector with " << rgb->size() << " elements." << std::endl;
                        throw;
                    }
                }
            if (refl != 0)
                try {
                    refl->push_back(refl_tmp);
                } catch (std::bad_alloc& ba){
                    std::cerr << "handle_line: Cannot add element to refl vector with " << refl->size() << " elements." << std::endl;
                    throw;
                }
            if (temp != 0)
                try {
                    temp->push_back(temp_tmp);
                } catch (std::bad_alloc& ba){
                    std::cerr << "handle_line: Cannot add element to temp vector with " << temp->size() << " elements." << std::endl;
                    throw;
                }
            if (ampl != 0)
                try {
                    ampl->push_back(ampl_tmp);
                } catch (std::bad_alloc& ba){
                    std::cerr << "handle_line: Cannot add element to ampl vector with " << ampl->size() << " elements." << std::endl;
                    throw;
                }
            if (type != 0)
                try {
                    type->push_back(type_tmp);
                } catch (std::bad_alloc& ba){
                    std::cerr << "handle_line: Cannot add element to type vector with " << type->size() << " elements." << std::endl;
                    throw;
                }
            if (devi != 0)
                try {
                    devi->push_back(devi_tmp);
                } catch (std::bad_alloc& ba){
                    std::cerr << "handle_line: Cannot add element to devi vector with " << devi->size() << " elements." << std::endl;
                    throw;
                }
            if (n != 0)
                for (int i = 0; i < 3; ++i) {
                    try {
                        n->push_back(n_tmp[i]);
                    } catch (std::bad_alloc& ba){
                        std::cerr << "handle_line: Cannot add element to n vector with " << n->size() << " elements." << std::endl;
                        throw;
                    }
                }
    }

    return true;
}

bool readASCII(std::istream& infile, IODataType* spec, ScanDataTransform& transform,
        PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned
        char>* rgb, std::vector<float>* refl, std::vector<float>* temp,
        std::vector<float>* ampl, std::vector<int>* type, std::vector<float>*
        devi, std::vector<double>* n, std::streamsize bufsize)
{
    /*
     * there seems to be no sane and fast way to read a file with multiple
     * whitespace separated values line by line without resorting to C functions
     *
     * we need the following:
     *   - split not only by more than one character (" " and "\t") but also by
     *     multiple characters together "\r\n"
     *   - allows to specify a maximum length to avoid reading in hundreds of
     *     megs
     *   - allow multiple subsequent separators without creating empty tokens
     *   - do not read into a new datastructure and thus waste time in
     *     allocating memory
     *
     * but:
     *   - std::getline fails because it only supports a single delimeter
     *   - type and not multiple: we need \n and \r\n
     *   - std::copy only reads into a vector and does not allow direct iteration
     *   - boost::split is horribly slow and leaves empty tokens
     *   - operator<< reads over newlines
     *
     * since nothing gives us what we want and is fast at the same time, we
     * roll our own solution...
     */

    unsigned int linenr = 1;
    char *buffer = (char *)malloc(bufsize);

    // if garbage is found at the top of the file, then we are liberal and
    // just skip over it. We allow up to 10 lines of garbage at the file top
    // to abort early and not print potentially millions of read errors.
    int header = 10;

    // we want to support \n and \r\n delimiters so it's okay to use
    // istream::getline to read a line (it supports a byte limit)
    // we then check whether the last character is a \r and remove it

    if (!checkSpec(spec, xyz, rgb, refl, temp, ampl, type, devi, n)) {
        std::cerr << "problems with spec" << std::endl;
        goto fail;
    }

    for (;;++linenr) {
        if (infile.eof()) break;

        try {
            infile.getline(buffer, bufsize, '\n');
        } catch(std::ios_base::failure e) {
            if (!infile.eof()) {
                std::cerr << "error reading a line in line " << linenr << std::endl;
                std::cerr << e.what() << std::endl;
                goto fail;
            } else {
                break;
            }
        }
        std::streamsize linelen = infile.gcount();
        // if failure but eof not reached, break
        if (infile.fail() && !infile.eof()) {
            std::cerr << "cannot find line ending within " << bufsize <<
                " characters and eof is not reached in line " << linenr << std::endl;
            break;
        }
        // if eof was not reached, then a terminator was found
        if (!infile.eof()) {
            linelen--;
        }
        // if the last character is \r replace it by \0
        if (linelen >= 1 &&
                buffer[linelen-1] == '\r' &&
                buffer[linelen] == '\0') {
            buffer[linelen-1] = '\0';
            linelen--;
        }

        if (!handle_line(buffer, linelen, linenr, spec, transform, filter, xyz, rgb, refl, temp, ampl, type, devi, n)) {
            std::cerr << "unable to parse line " << linenr << std::endl;
            // A line contained an error, so we decrement the header variable
            header -= 1;
            // If we decrement too much, we start quit with an error
            if (header < 0) {
                goto fail;
            }
        } else if (header >= 0) {
            // A line was successfully read. This means the header is over and
            // no more errors must follow.
            header = -1;
        }
    }

    if (infile.bad() && !infile.eof()) {
        perror("error while reading file");
        goto fail;
    }
    free(buffer);
    return true;
fail:
    free(buffer);
    return false;
}

/* a helper used by open_path and open_path writing. It goes through a path
 * from root downward and if it encounters a component that is not a
 * directory, it will pass this location plus the remainder to the handler
 * function */
bool find_path_archive(boost::filesystem::path data_path, std::function<bool (boost::filesystem::path, boost::filesystem::path)> handler)
{
    boost::filesystem::path archivepath;
    // go through all components
    for(auto part = data_path.begin(); part != data_path.end(); ++part) {
        archivepath /= *part;
        if (boost::filesystem::is_directory(archivepath))
            continue;
        if (!exists(archivepath))
            continue;
        // if the current component was not a directory, try to
        // open it as a file and try to find the remaining path
        // in it
        boost::filesystem::path remainder;
        // strip off the part of the path that is the archive name
        ++part;
        for (; part != data_path.end(); ++part)
            remainder /= *part;
        // now try opening the path as an archive
        return handler(archivepath, remainder);
    }
    return false;
}

/*
 * open a path for reading such that part of the path can also be inside an archive
 */
bool open_path(boost::filesystem::path data_path, std::function<bool (std::istream &)> handler)
{
    bool ret = 0;
    if (exists(data_path)) {
        boost::filesystem::ifstream data_file(data_path);
        ret = handler(data_file);
#ifdef WITH_LIBZIP
    } else {
        ret = find_path_archive(data_path, [=,&handler](boost::filesystem::path archivepath, boost::filesystem::path remainder) -> bool {
            /* open the archive for reading */
            int error;
            int flags = 0;
            struct zip *archive = zip_open(archivepath.string().c_str(), flags, &error);
            if (archive == nullptr) {
                // FIXME: the following changed with libzip 1.0
                //char buf[128]{};
                //zip_error_to_str(buf, sizeof (buf), error, errno);
                //throw std::runtime_error(buf);
                throw std::runtime_error("zip_open failed");
            }
            zip_int64_t idx = zip_name_locate(archive, remainder.string().c_str(), 0);
            /* check if the file cannot be found */
            if (idx == -1)
                return false;
            struct zip_file *zipfile = zip_fopen_index(archive, idx, 0);
            std::stringstream ss( std::ios_base::out | std::ios_base::in | std::ios_base::binary );
            //ssize_t bufsize = 4096;
            zip_int64_t bufsize = 4096;
            char *buf = (char *)malloc(bufsize);
            do {
                zip_int64_t rb = zip_fread(zipfile, buf, bufsize);
                if (rb == -1)
                    throw std::runtime_error("cannot zip_fread");
                ss.write(buf, rb);
                // a short read means, that there is not more to read
                if (rb != bufsize)
                    break;
            } while (true);
            bool ret = handler(ss);
            // FIXME: check return values of the following two calls
            zip_fclose(zipfile);
            zip_close(archive);
            free(buf);
            return ret;
        });
#endif
    }
    if (!ret) {
        std::cerr << "Path does neither exist nor is a zip archive: " << data_path << std::endl;
    }
    return ret;
}

bool open_path_writing(boost::filesystem::path data_path, std::function<bool (std::ostream &)> handler)
{
    if (exists(data_path)) {
        boost::filesystem::ofstream data_file(data_path);
        return handler(data_file);
    }

#ifdef WITH_LIBZIP
    return find_path_archive(data_path, [=,&handler](boost::filesystem::path archivepath, boost::filesystem::path remainder) -> bool {
            /* open the archive for reading */
            int error;
            int flags = 0;
            std::stringstream ss( std::ios_base::out | std::ios_base::in | std::ios_base::binary );
            if (!handler(ss))
                return false;
            struct zip *archive = zip_open(archivepath.string().c_str(), flags, &error);
            if (archive == nullptr) {
                // FIXME: the following changed with libzip 1.0
                //char buf[128]{};
                //zip_error_to_str(buf, sizeof (buf), error, errno);
                throw std::runtime_error("zip_open failed");
            }
            std::string data = ss.str();
            struct zip_source *source = zip_source_buffer(archive, data.c_str(), data.length(), 0);
            if (source == nullptr) {
                // FIXME: the following changed with libzip 1.0
                //char buf[128]{};
                //zip_error_to_str(buf, sizeof (buf), error, errno);
                //throw std::runtime_error(buf);
                throw std::runtime_error("zip_source_buffer_create failed");
            }
            zip_int64_t idx = zip_name_locate(archive, remainder.string().c_str(), 0);
            if (idx == -1) {
#ifdef LIBZIP_OLD
                zip_int64_t newidx = zip_add(archive, remainder.string().c_str(), source);
#else
                zip_int64_t newidx = zip_file_add(archive, remainder.string().c_str(), source, 0);
#endif
                if (newidx == -1)
                    throw std::runtime_error("zip_file_add failed");
            } else {
#ifdef LIBZIP_OLD
                int ret = zip_replace(archive, idx, source);
#else
                int ret = zip_file_replace(archive, idx, source, 0);
#endif
                if (ret == -1)
                    throw std::runtime_error("zip_file_replace failed");
            }
            zip_close(archive);
            return true;
        });
#else
    return 0;
#endif
}

#ifdef WITH_LIBZIP
bool write_multiple(std::map<std::string,std::string> contentmap)
{
    std::map<std::string, struct zip *> archivehandles;
    for (auto it=contentmap.begin(); it != contentmap.end(); ++it) {
        std::string path = it->first;
        std::string content = it->second;

        // check if the directory part of the frames file exists and is a
        // directory
        boost::filesystem::path dirname = boost::filesystem::path(path).parent_path();
        if (boost::filesystem::is_directory(dirname)) {
            boost::filesystem::ofstream data_file(path);
            data_file << content;
            data_file.close();
            continue;
        }

        bool ret = find_path_archive(path, [=,&archivehandles](boost::filesystem::path archivepath, boost::filesystem::path remainder) -> bool {
            auto ah_it = archivehandles.find(archivepath.string());
            if (ah_it == archivehandles.end()) {
                int error;
                int flags = 0;
                struct zip *archive = zip_open(archivepath.string().c_str(), flags, &error);
                if (archive == nullptr) {
                    // FIXME: the following changed with libzip 1.0
                    //char buf[128]{};
                    //zip_error_to_str(buf, sizeof (buf), error, errno);
                    throw std::runtime_error("zip_open failed");
                }
                ah_it = archivehandles.insert(std::pair<std::string, struct zip *>(archivepath.string(), archive)).first;
            }
            struct zip *archive = ah_it->second;
            struct zip_source *source = zip_source_buffer(archive, content.c_str(), content.length(), 0);
            if (source == nullptr) {
                // FIXME: the following changed with libzip 1.0
                //char buf[128]{};
                //zip_error_to_str(buf, sizeof (buf), error, errno);
                //throw std::runtime_error(buf);
                throw std::runtime_error("zip_source_buffer_create failed");
            }
            zip_int64_t idx = zip_name_locate(archive, remainder.string().c_str(), 0);
            if (idx == -1) {
#ifdef LIBZIP_OLD
                zip_int64_t newidx = zip_add(archive, remainder.string().c_str(), source);
#else
                zip_int64_t newidx = zip_file_add(archive, remainder.string().c_str(), source, 0);
#endif
                if (newidx == -1)
                    throw std::runtime_error("zip_file_add failed");
            } else {
#ifdef LIBZIP_OLD
                int ret = zip_replace(archive, idx, source);
#else
                int ret = zip_file_replace(archive, idx, source, 0);
#endif
                if (ret == -1)
                    throw std::runtime_error("zip_file_replace failed");
            }
            return true;
        });
        if (!ret) {
            throw std::runtime_error("cannot find zip archive component in path");
        }
    }

    for (auto it=archivehandles.begin(); it != archivehandles.end(); ++it) {
        zip_close(it->second);
    }

    return true;
}
#endif

/* vim: set ts=4 sw=4 et: */
