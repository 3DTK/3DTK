/*** E57ExceptionFunctions.cpp example: get info about E57Exceptions */
//! @file
//! @brief example: get info about E57Exceptions
#include <iostream>
#include "E57Foundation.h"
using namespace e57;
using namespace std;

//! @brief Example use of E57Exception functions
int main(int /*argc*/, char** /*argv*/) {
    const char* fileName = "doesnt_exist.e57";
    try {
        ImageFile imf(fileName, "r");
        imf.close(); // don't forget to explicitly close the ImageFile
    } catch(E57Exception& ex) {
        cout << "errorCode   = " << ex.errorCode() << endl;
        cout << "errorString = " << E57Utilities().errorCodeToString(ex.errorCode()).c_str() << endl;
        cout << "context     = " << ex.context()   << endl;
        cout << "what        = " << ex.what()      << endl;

        cout << "sourceFileName     = " << ex.sourceFileName()     << endl;
        cout << "sourceFunctionName = " << ex.sourceFunctionName() << endl;
        cout << "sourceLineNumber   = " << ex.sourceLineNumber()   << endl;

        cout << endl;
        ex.report(__FILE__, __LINE__, __FUNCTION__);

        cout << endl;
        if (ex.errorCode() == E57_ERROR_OPEN_FAILED)
            cout << "File " << fileName << " not found." << endl;

        return(-1);
    }
    return(0);
}
