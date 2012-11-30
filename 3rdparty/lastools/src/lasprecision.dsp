# Microsoft Developer Studio Project File - Name="lasprecision" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=lasprecision - Win32 Debug with GUI
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "lasprecision.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "lasprecision.mak" CFG="lasprecision - Win32 Debug with GUI"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "lasprecision - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "lasprecision - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE "lasprecision - Win32 Release With GUI" (based on "Win32 (x86) Console Application")
!MESSAGE "lasprecision - Win32 Debug with GUI" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /w /W0 /GX /O2 /I "..\laslib\inc" /D "NDEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /i "../src" /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 ../laslib/lib/LASlib.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Release\lasprecision.exe ..\bin\lasprecision.exe
# End Special Build Tool

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /w /W0 /Gm /GX /ZI /Od /I "..\laslib\inc" /D "_DEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /i "..\..\src" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ../laslib/libD/LASlib.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Debug\lasprecision.exe ..\bin\lasprecision.exe
# End Special Build Tool

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "lasprecision___Win32_Release_With_GUI"
# PROP BASE Intermediate_Dir "lasprecision___Win32_Release_With_GUI"
# PROP BASE Ignore_Export_Lib 0
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /w /W0 /GX /O2 /I "..\laslib\inc" /D "NDEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /w /W0 /GX /O2 /I "..\laslib\inc" /I "..\src" /I "..\src_full\glui_api" /I "..\src_full\glut_api" /D "NDEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /D "COMPILE_WITH_GUI" /D "COMPILE_WITH_MULTI_CORE" /YX /FD /c
# ADD BASE RSC /l 0x409 /i "../src" /d "NDEBUG"
# ADD RSC /l 0x409 /i "../src" /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ../laslib/lib/LASlib.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 ../laslib/lib/LASlib.lib  ../src_full/glui_api/glui32.lib ../src_full/glut_api/glutstatic.lib ../src_full/glut_api/glut32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Release\lasprecision.exe ..\bin\lasprecision.exe
# End Special Build Tool

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "lasprecision___Win32_Debug_with_GUI"
# PROP BASE Intermediate_Dir "lasprecision___Win32_Debug_with_GUI"
# PROP BASE Ignore_Export_Lib 0
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /w /W0 /Gm /GX /ZI /Od /I "..\laslib\inc" /D "_DEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /w /W0 /Gm /GX /ZI /Od /I "..\laslib\inc" /I "..\src" /I "..\src_full\glui_api" /I "..\src_full\glut_api" /D "_DEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /D "COMPILE_WITH_GUI" /D "COMPILE_WITH_MULTI_CORE" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /i "..\..\src" /d "_DEBUG"
# ADD RSC /l 0x409 /i "..\..\src" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ../laslib/libD/LASlib.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ../laslib/libD/LASlib.lib  ../src_full/glui_api/glui32.lib ../src_full/glut_api/glutstatic.lib ../src_full/glut_api/glut32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# Begin Special Build Tool
SOURCE="$(InputPath)"
PostBuild_Cmds=copy Debug\lasprecision.exe ..\bin\lasprecision.exe
# End Special Build Tool

!ENDIF 

# Begin Target

# Name "lasprecision - Win32 Release"
# Name "lasprecision - Win32 Debug"
# Name "lasprecision - Win32 Release With GUI"
# Name "lasprecision - Win32 Debug with GUI"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\geoprojectionconverter.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\laslicense.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lasoccupancy.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src\lasprecision.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lasprecision_gui.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lasprecision_multi_core.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lastools_gui.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lastools_multi_core.cpp

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=..\laslib\inc\lasdefinitions.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\lasfilter.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\lasreader.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\lasutility.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\laswaveform13reader.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\laswaveform13writer.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\laswriter.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\laszip.hpp
# End Source File
# Begin Source File

SOURCE=..\laslib\inc\mydefs.hpp
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# Begin Source File

SOURCE=..\src_full\icon1.ico

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=..\src_full\lastool.rc

!IF  "$(CFG)" == "lasprecision - Win32 Release"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug"

# PROP Exclude_From_Build 1

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Release With GUI"

!ELSEIF  "$(CFG)" == "lasprecision - Win32 Debug with GUI"

!ENDIF 

# End Source File
# End Group
# End Target
# End Project
