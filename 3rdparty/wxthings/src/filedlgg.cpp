//////////////////////////////////////////////////////////////////////////////
// Name:        filedlgg.cpp
// Purpose:     wxGenericFileDialog stub to include the wxWidget's source file by the same name
// Author:      John Labenski
// Modified by:
// Created:     12/12/98
// RCS-ID:
// Copyright:   (c) John Labenski
// Licence:     wxWindows licence
/////////////////////////////////////////////////////////////////////////////

// wxWidgets does not compile wxWidgets/src/generic/filedlgg.cpp into the
// library in MSW. In Linux/OSX wxWidgets does use the generic filedialog.
// This file will include the source in wxWidgets/src/generic/filedlgg.cpp
// only on platforms that do not already have it compiled into the wxWidgets lib.

// NOTE: MSVC cannot use environment var $(WXWIN)/src/generic/filedlgg.cpp in
// their project files to allow you to compile the generic file dialog.
// You have to specify the relative or absolute path to the wxWidgets
// distribution which is annoying since everyone will probably have it in a
// different place. By #including the source from this file we can avoid this
// and use the $(WXWIN) environment var in the -I include path.

#include "precomp.h"

#include "wx/things/thingdef.h"

#include <wx/defs.h>
#include <wx/filedlg.h>          // Will include "wx/generic/filedlgg.h" on platforms that use it
#include <wx/volume.h>           // wxFSVolumeBase
#include "wx/generic/dirctrlg.h" // wxFileIconsTable

// We don't need to #include this file here
//#include "wx/generic/filedlgg.h"
// and if we didn't include the generic filedlgg.h then include the source here

#ifndef _WX_FILEDLGG_H_ // header guard of "wx/generic/filedlgg.h" 

    // Typically we have $(WXWIN)/include in the search path so this should
    // find the path to filedlgg correctly.
    #include "../src/generic/filedlgg.cpp"

// --------------------------------------------------------------------------
// DO NOT DELETE : Cmake will configure this file and replace the marker below.
// @CMAKE_CONFIGURE_WXTHINGS_FILEDLGG@


#endif //_WX_FILEDLGG_H_
