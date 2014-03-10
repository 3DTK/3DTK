#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <climits>
#include "scanio/helper.h"
#include "slam6d/globals.icc"

using namespace boost::filesystem;

bool ScanDataTransform_identity::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi)
{
    return true;
}

bool ScanDataTransform_ks::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi)
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

bool ScanDataTransform_riegl::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi)
{
    double tmp;
    tmp = xyz[2];
    xyz[2] = 100.0 * xyz[0];
    xyz[0] = -100.0 * xyz[1];
    xyz[1] = 100.0 * tmp;

    return true;
}

bool ScanDataTransform_rts::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi)
{
    // check if TYPE_INVALID flag for rts invalid points is set
    if(*type & 0x10)
        return false;

    double tmp;
    tmp = xyz[2];
    xyz[2] = 0.1 * xyz[0];
    xyz[0] = -0.1 * xyz[1];
    xyz[1] = 0.1 * tmp;

    return true;
}

bool ScanDataTransform_xyz::transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi)
{
    double tmp;
    tmp = xyz[2];
    xyz[2] = xyz[0];
    xyz[0] = -xyz[1];
    xyz[1] = tmp;

    return true;
}


std::list<std::string> readDirectoryHelper(const char *dir_path,
        unsigned int start,
        unsigned int end,
        const char **data_path_suffixes,
        const char *data_path_prefix)
{
    std::list<std::string> identifiers;
    for (unsigned int i = start; i <= end; ++i) {
        // identifier is /d/d/d (000-999)
        std::string identifier(to_string(i, 3));
        // scan consists of data and pose files
        bool found = false;
        for (const char **s = data_path_suffixes; *s != 0; s++) {
            path data(dir_path);
            data /= path(std::string(data_path_prefix) + identifier + *s);
            if (exists(data)) {
                found = true;
                break;
            }
        }
        // stop if part of a scan is missing or end by absence is detected
        if (!found) {
            std::cerr << "No data found for " << data_path_prefix << identifier << "!" << endl;
            break;
        }
        identifiers.push_back(identifier);
    }
    return identifiers;
}

void readPoseHelper(const char *dir_path,
        const char *identifier,
        double *pose,
        const char *pose_path_suffix,
        const char *pose_path_prefix)
{
    unsigned int i;

    path pose_path(dir_path);
    pose_path /= path(std::string(pose_path_prefix) + identifier +
            pose_path_suffix);
    if (!exists(pose_path)) {
        throw std::runtime_error(std::
                string("There is no pose file for [") +
                identifier + "] in [" + dir_path + "]");
    }

    // open pose file
    ifstream pose_file(pose_path);

    // if the file is open, read contents
    if (pose_file.good()) {
        // read 6 plain doubles
        for (i = 0; i < 6; ++i)
            pose_file >> pose[i];
        pose_file.close();

        // convert angles from deg to rad
        for (i = 3; i < 6; ++i)
            pose[i] = rad(pose[i]);
    } else {
        throw std::runtime_error(std::
                string
                ("Pose file could not be opened for [") +
                identifier + "] in [" + dir_path + "]");
    }
}

bool uosHeaderTest(char *line)
{
    char *cur, *old;
    old = line;
    // start with one or more digits
    for (cur = old; isdigit(*cur); ++cur);
    // check if anything was read in
    if (cur == old)
        return false;
    old = cur;
    // follow with one space, one 'x' and one space
    if (old[0] == '\0' || old[0] != ' '
            || old[1] == '\0' || old[1] != 'x'
            || old[2] == '\0' || old[2] != ' ')
        return false;
    old+=3;
    // end with one or more digits
    for (cur = old; isdigit(*cur); ++cur);
    // check if anything was read in
    if (cur == old)
        return false;
    // check if the last character is the end of the line
    if (cur[0] != '\0' && (cur[0] != '\r' || cur[1] != '\0'))
        return false;
    return true;
}

void uosHeaderTest(std::istream& infile, std::streamsize bufsize)
{
    char *buffer = (char *)malloc(bufsize);
    std::streampos sp = infile.tellg();
    try { 
        infile.getline(buffer, bufsize, '\n');
    } catch(std::ios::failure e) {
        std::cerr << "error reading first line" << endl;
        std::cerr << "did not find a newline after " << bufsize << " characters" << endl;
        std::cerr << e.what() << endl;
        return;
    }
    // if uos header, skip it, otherwise reset
    if (!uosHeaderTest(buffer)) {
        infile.seekg(sp);
    }
    free(buffer);
}

bool strtoval(char *pos, unsigned int linenr, double* ret)
{
    char *endptr;
    errno = 0;
    double val = strtod(pos, &endptr);
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << endl;
        if (val == HUGE_VAL) {
            std::cerr << "overflow" << endl;
        } else if (val == 0) {
            std::cerr << "underflow" << endl;
        }
        perror("strod");
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, float* ret)
{
    char *endptr;
    errno = 0;
    float val = strtof(pos, &endptr);
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << endl;
        if (val == HUGE_VALF) {
            std::cerr << "overflow" << endl;
        } else if (val == 0) {
            std::cerr << "underflow" << endl;
        }
        perror("strof");
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, unsigned char* ret)
{
    char *endptr;
    errno = 0;
    long val = strtol(pos, &endptr, 10);
    if (errno != 0 && val == 0) {
        std::cerr << "error in line " << linenr << endl;
        perror("strol");
        return false;
    }
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << endl;
        if (val < 0)
            std::cerr << "cannot be smaller than 0" << endl;
        if (val > 255)
            std::cerr << "cannot be greater than 255" << endl;
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << endl;
        return false;
    }
    *ret = val;
    return true;
}

bool strtoval(char *pos, unsigned int linenr, int* ret)
{
    char *endptr;
    errno = 0;
    long val = strtol(pos, &endptr, 10);
    if (errno != 0 && val == 0) {
        std::cerr << "error in line " << linenr << endl;
        perror("strol");
        return false;
    }
    if (errno == ERANGE) {
        std::cerr << "error in line " << linenr << endl;
        if (val < INT_MIN)
            std::cerr << "cannot be smaller than " << INT_MIN << endl;
        if (val > INT_MAX)
            std::cerr << "cannot be greater than " << INT_MAX << endl;
        return false;
    }
    if (pos == endptr) {
        std::cerr << "no conversion performed in line " << linenr << endl;
        return false;
    }
    if (*endptr != '\0') {
        std::cerr << "found garbage in line " << linenr << endl;
        return false;
    }
    *ret = val;
    return true;
}

bool storeval(char *pos, unsigned int linenr, IODataType currspec, double* xyz, int* xyz_idx, unsigned char* rgb, int* rgb_idx, float* refl, float* temp, float* ampl, int* type, float* devi)
{
    switch (currspec) {
        case DATA_XYZ:
            return strtoval(pos, linenr, &xyz[(*xyz_idx)++]);
        case DATA_RGB:
            return strtoval(pos, linenr, &rgb[(*rgb_idx)++]);
        case DATA_REFLECTANCE:
            return strtoval(pos, linenr, refl);
        case DATA_TEMPERATURE:
            return strtoval(pos, linenr, devi);
        case DATA_AMPLITUDE:
            return strtoval(pos, linenr, ampl);
        case DATA_TYPE:
            return strtoval(pos, linenr, type);
        case DATA_DEVIATION:
            return strtoval(pos, linenr, devi);
        case DATA_DUMMY:
            return true;
        case DATA_TERMINATOR:
            std::cerr << "too many values in line " << linenr << endl;
            return false;
        default:
            return false;
    }
}

bool checkSpec(IODataType* spec, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* refl, std::vector<float>* temp, std::vector<float>* ampl, std::vector<int>* type, std::vector<float>* devi)
{
    int count = 0;
    int xyzcount = 0;
    int rgbcount = 0;
    int reflcount = 0;
    int tempcount = 0;
    int amplcount = 0;
    int typecount = 0;
    int devicount = 0;
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
            case DATA_DUMMY:
                break;
            default:
                std::cerr << "unknown spec: " << *currspec;
                return false;
        }
    }
    if (count == 0) {
        std::cerr << "must supply more than zero specs" << endl;
        return false;
    }
    // check if the spec matches the supplied vectors
    if (xyz == 0 && xyzcount != 0) {
        std::cerr << "you gave a xyz spec but no xyz vector" << endl;
        return false;
    }
    if (xyz != 0 && xyzcount != 3) {
        std::cerr << "you gave a xyz vector, so you must supply exactly three xyz specs" << endl;
        return false;
    }
    if (rgb == 0 && rgbcount != 0) {
        std::cerr << "you gave a rgb spec but no rgb vector" << endl;
        return false;
    }
    if (rgb != 0 && rgbcount != 3) {
        std::cerr << "you gave a rgb vector, so you must supply exactly three rgb specs" << endl;
        return false;
    }
    if (refl == 0 && reflcount != 0) {
        std::cerr << "you gave a reflection spec but no reflection vector" << endl;
        return false;
    }
    if (refl != 0 && reflcount != 1) {
        std::cerr << "you gave a reflection vector, so you must supply exactly one reflection spec" << endl;
        return false;
    }
    if (temp == 0 && tempcount != 0) {
        std::cerr << "you gave a temperature spec but no temperature vector" << endl;
        return false;
    }
    if (temp != 0 && tempcount != 1) {
        std::cerr << "you gave a temperature vector, so you must supply exactly one temperature spec" << endl;
        return false;
    }
    if (ampl == 0 && amplcount != 0) {
        std::cerr << "you gave an amplitude spec but no amplitude vector" << endl;
        return false;
    }
    if (ampl != 0 && amplcount != 1) {
        std::cerr << "you gave an amplitude vector, so you must supply exactly one amplitude spec" << endl;
        return false;
    }
    if (type == 0 && typecount != 0) {
        std::cerr << "you gave a type spec but no type vector" << endl;
        return false;
    }
    if (type != 0 && typecount != 1) {
        std::cerr << "you gave a type vector, so you must supply exactly one type spec" << endl;
        return false;
    }
    if (devi == 0 && devicount != 0) {
        std::cerr << "you gave a deviation spec but no deviation vector" << endl;
        return false;
    }
    if (devi != 0 && devicount != 1) {
        std::cerr << "you gave a deviation vector, so you must supply exactly one deviation spec" << endl;
        return false;
    }
    return true;
}

bool readASCII(std::istream& infile, IODataType* spec, ScanDataTransform& transform, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* refl, std::vector<float>* temp, std::vector<float>* ampl, std::vector<int>* type, std::vector<float>* devi, std::streamsize bufsize)
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
    // temporary storage areas
    double xyz_tmp[3];
    int xyz_idx = 0;
    unsigned char rgb_tmp[3];
    int rgb_idx = 0;
    float refl_tmp, temp_tmp, ampl_tmp, devi_tmp;
    int type_tmp;


    char *buffer = (char *)malloc(bufsize);

    // we want to support \n and \r\n delimiters so it's okay to use
    // istream::getline to read a line (it supports a byte limit)
    // we then check whether the last character is a \r and remove it

    IODataType *currspec = spec;

    if (!checkSpec(spec, xyz, rgb, refl, temp, ampl, type, devi)) {
        std::cerr << "problems with spec" << endl;
        goto fail;
    }

    for (unsigned int linenr = 1;;++linenr) {
        try { 
            infile.getline(buffer, bufsize, '\n');
        } catch(std::ios::failure e) {
            if (!infile.eof()) {
                std::cerr << "error reading a line in line " << linenr << endl;
                std::cerr << e.what() << endl;
                goto fail;
            } else {
                break;
            }
        }
        char *pos = buffer;
        std::streamsize linelen = infile.gcount();
        // if failure but eof not reached, break
        if (infile.fail() && !infile.eof()) {
            std::cerr << "cannot find line ending within " << bufsize <<
                " characters and eof is not reached in line " << linenr << endl;
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
        // skip over leading whitespace
        for (; *pos == ' ' || *pos == '\t'; ++pos, --linelen);
        // skip this line if it is empty and exit if it's empty and the last
        if (linelen == 0) {
            if (infile.eof()) break;
            else continue;
        }
        // skip the line if it starts with the comment character
        if (*pos == '#')
            continue;

        char *cur;
        // now go through all fields and handle them according to the spec
        for (cur = pos; *cur != '\0' && *cur != '#'; ++cur) {
            // skip over everything that is not part of a field
            if (*cur != ' ' && *cur != '\t')
                continue;
            // we found the end of a field so lets read its content
            *cur = '\0';
            if (!storeval(pos, linenr, *currspec, xyz_tmp, &xyz_idx, rgb_tmp,
                        &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp))
                goto fail;
            currspec++;
            // read in the remaining whitespace
            pos = cur + 1;
            for (; *pos == ' ' || *pos == '\t'; ++pos);
            cur = pos - 1;
        }
        // read in last value (if any)
        if (*pos != '#' && *pos != '\0') {
            *cur = '\0';
            // read in the last value
            if (!storeval(pos, linenr, *currspec, xyz_tmp, &xyz_idx, rgb_tmp,
                        &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp))
                goto fail;
            // check if more values were expected
            currspec++;
        }
        if (*currspec != DATA_TERMINATOR) {
            std::cerr << "less values than in spec in line " << linenr << endl;
            goto fail;
        }
        // check if three values were read in for xyz and rgb 
        if (xyz != 0 && xyz_idx != 3) {
            std::cerr << "can't understand " << xyz_idx << " coordinate values in line " << linenr << endl;
            goto fail;
        }
        if (rgb != 0 && rgb_idx != 3) {
            std::cerr << "can't understand " << xyz_idx << " color values in line " << linenr << endl;
            goto fail;
        }
        // apply transformations and filter data and append to vectors
        if (transform.transform(xyz_tmp, rgb_tmp, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp)
                && (xyz == 0 || filter.check(xyz_tmp)) ) {
            if (xyz != 0)
                for (int i = 0; i < 3; ++i) xyz->push_back(xyz_tmp[i]);
            if (rgb != 0)
                for (int i = 0; i < 3; ++i) rgb->push_back(rgb_tmp[i]);
            if (refl != 0)
                refl->push_back(refl_tmp);
            if (temp != 0)
                temp->push_back(temp_tmp);
            if (ampl != 0)
                ampl->push_back(ampl_tmp);
            if (type != 0)
                type->push_back(type_tmp);
            if (devi != 0)
                devi->push_back(devi_tmp);
        }
        // reset current spec
        currspec = spec;
        // reset indices
        xyz_idx = 0;
        rgb_idx = 0;
    }

    if (infile.bad()) {
        perror("error while reading file");
        goto fail;
    }
    free(buffer);
    return true;
fail:
    free(buffer);
    return false;
}
