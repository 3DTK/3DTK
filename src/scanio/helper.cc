#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <climits>
#include "scanio/helper.h"
#include "slam6d/globals.icc"
#include "archive_reader.hpp"

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
    xyz[0] = 0.1 * xyz[1];
    xyz[1] = -0.1 * tmp;

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

bool pathExistsDummy(std::istream &data_file, PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation) {
    return true;
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
            if (open_path(data, filter, NULL, NULL, NULL, NULL, NULL, NULL, NULL, pathExistsDummy)) {
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

    boost::filesystem::path pose_path(dir_path);
    pose_path /= boost::filesystem::path(std::string(pose_path_prefix) + identifier +
            pose_path_suffix);
    if (!boost::filesystem::exists(pose_path)) {
        throw std::runtime_error(std::
                string("There is no pose file for [") +
                identifier + "] in [" + dir_path + "]");
    }

    // open pose file
    boost::filesystem::ifstream pose_file(pose_path);

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

std::streamsize uosHeaderTest(std::istream& infile, char **line, std::streamsize bufsize)
{
    /*
     * in case the first line turned out not to be a uos header, this function
     * originally just did a seekg() back to the beginning of the file. With
     * the integration of libarchive this is not possible anymore because
     * seeking backwards is not possible. So instead, if the first line is
     * found not to be a header but actual data, the allocated buffer is
     * returned so that it can later be passed to readACSII as the first line
     * of the input.
     *
     * An alternative to doing this would be to wrap the istream in another
     * istream which supports buffering of the last X bytes read so that it is
     * possible to seek back in the stream a couple of bytes. But implementing
     * a streambuf class that does this, was deemed to complicated and hence
     * this solution.
     *
     * Should there ever exist such a wrapper class, then it can be used to
     * answer this question: http://stackoverflow.com/questions/31478256/
     */
    char *buffer = (char *)malloc(bufsize);
    std::streampos sp = infile.tellg();
    try {
        infile.getline(buffer, bufsize, '\n');
    } catch(std::ios::failure e) {
        std::cerr << "error reading first line" << endl;
        std::cerr << "did not find a newline after " << bufsize << " characters" << endl;
        std::cerr << e.what() << endl;
        free(buffer);
        *line = NULL;
        return -1;
    }
    std::streamsize linelen = infile.gcount();
    // if failure but eof not reached, break
    if (infile.fail() && !infile.eof()) {
        std::cerr << "cannot find line ending within " << bufsize <<
            " characters and eof is not reached in line 1"<< endl;
        free(buffer);
        *line = NULL;
        return -1;
    }
    // if eof was not reached, then a terminator was found
    // strip it off
    if (!infile.eof()) {
        buffer[linelen-1] = '\0';
        linelen--;
    }
    // if the last character is \r replace it by \0
    if (linelen >= 1 &&
            buffer[linelen-1] == '\r' &&
            buffer[linelen] == '\0') {
        buffer[linelen-1] = '\0';
        linelen--;
    }
    // the uos header must be at least five bytes because it is of the form
    // A x B
    // if no uos header or comment was found, then the first line was actual
    // data which we return
    if (linelen < 5 || !uosHeaderTest(buffer)) {
        *line = buffer;
        return linelen;
    }
    // otherwise, the first line was the header which we discard
    free(buffer);
    *line = NULL;
    return 0;
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
            std::cerr << "storeval failed at " << linenr << endl;
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

bool handle_line(char *pos, std::streamsize linelen, unsigned int linenr, IODataType *currspec,
ScanDataTransform& transform, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned
        char>* rgb, std::vector<float>* refl, std::vector<float>* temp,
        std::vector<float>* ampl, std::vector<int>* type, std::vector<float>*
        devi)
{
    // temporary storage areas
    double xyz_tmp[3];
    unsigned char rgb_tmp[3];
    float refl_tmp, temp_tmp, ampl_tmp, devi_tmp;
    int type_tmp;
    int xyz_idx = 0;
    int rgb_idx = 0;

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
                    &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp))
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
                    &rgb_idx, &refl_tmp, &temp_tmp, &ampl_tmp, &type_tmp, &devi_tmp))
            return false;
        // check if more values were expected
        currspec++;
    }
    if (*currspec != DATA_TERMINATOR) {
        std::cerr << "less values than in spec in line " << linenr << endl;
        return false;
    }
    // check if three values were read in for xyz and rgb 
    if (xyz != 0 && xyz_idx != 3) {
        std::cerr << "can't understand " << xyz_idx << " coordinate values in line " << linenr << endl;
        return false;
    }
    if (rgb != 0 && rgb_idx != 3) {
        std::cerr << "can't understand " << xyz_idx << " color values in line " << linenr << endl;
        return false;
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

    return true;
}

bool readASCII(std::istream& infile, char *firstline, std::streamsize
        lenfirstline, IODataType* spec, ScanDataTransform& transform,
        PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned
        char>* rgb, std::vector<float>* refl, std::vector<float>* temp,
        std::vector<float>* ampl, std::vector<int>* type, std::vector<float>*
        devi, std::streamsize bufsize)
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

    // we want to support \n and \r\n delimiters so it's okay to use
    // istream::getline to read a line (it supports a byte limit)
    // we then check whether the last character is a \r and remove it

    if (!checkSpec(spec, xyz, rgb, refl, temp, ampl, type, devi)) {
        std::cerr << "problems with spec" << endl;
        goto fail;
    }

    if (firstline != NULL) {
        if (!handle_line(firstline, lenfirstline, linenr, spec, transform, filter, xyz, rgb, refl, temp, ampl, type, devi))
            goto fail;
        ++linenr;
    }

    for (;;++linenr) {
        if (infile.eof()) break;

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

        if (!handle_line(buffer, linelen, linenr, spec, transform, filter, xyz, rgb, refl, temp, ampl, type, devi))
            goto fail;
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

bool open_path(boost::filesystem::path data_path, PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation, bool (*handler)(std::istream &,
            PointFilter&, std::vector<double>*, std::vector<unsigned char>*,
            std::vector<float>*, std::vector<float>*, std::vector<float>*,
            std::vector<int>*, std::vector<float>*))
{
    if (exists(data_path)) {
        boost::filesystem::ifstream data_file(data_path);
        return handler(data_file, filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation);
    } else {
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
            boost::filesystem::ifstream fs(archivepath);
            ns_archive::reader reader = ns_archive::reader::make_reader<ns_archive::ns_reader::format::_ALL, ns_archive::ns_reader::filter::_ALL>(fs, 10240);
            for(auto e : reader)
            {
                if (e->get_header_value_pathname() == remainder) {
                    istream &stream(e->get_stream());
                    return handler(stream, filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation);
                }
            }
            break;
        }
        return false;
    }
}

/* vim: set ts=4 sw=4 et: */
