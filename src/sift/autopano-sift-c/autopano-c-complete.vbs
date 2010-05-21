 ' autopano-c-complete.vbs
 ' drop in replacement for autopano-c-complete.sh
 ' based on by Pablo d'Angelo <pablo.dangelo@web.de>
 ' copyright 2008 Yuval Levy <http://www.levy.ch/>
 ' licensed under GPL v2
 ' additional contributors:
 ' Pete Holzmann (wildcard support, nicer output)
 ' functions and other code tidbits licensed from:
 ' Christian d'Heureuse <www.source-code.biz>
 
 ' names of the autopano-sift-c executables
 generatekeys = "generatekeys.exe"
 autopano = "autopano.exe"
 ' name of the directory where they are normally installed
 stdinstalldir = "C:\Program Files\autopano-sift-C\bin"
 
 Set arguments = WScript.Arguments
 
 ' describe usage
 Function usage()
 	a = "usage: autopano-c-complete.vbs [options] -o panoproject.pto image1 image2 [...]" & VbCrLf _
 	 & VbCrLf & "options can be:" _
 	 & VbCrLf &   "  -o | --output   name    filename of created panorama project" _
 	 & VbCrLf &   "  -s | --size     number  downsize images until width and height is" _
 	 & VbCrLf &   "                          smaller than number, default 700" _
 	 & VbCrLf &   "  -p | --points   number  number of generated control points between," _
 	 & VbCrLf &   "                          each pair, default: 10" _
 	 & VbCrLf &   "  -n | --noransac         no ransac detection, useful for fisheye images" _
 	 & VbCrLf &   "  -c | --clean            do not reuse keypoints detected in earlier runs," _
 	 & VbCrLf &   "                          deletes old keypoint files." _
 	 & VbCrLf &   "image1 can be a wildcard, e.g. *.jpg"
    WScript.Echo a
 	WScript.Quit 1
 End Function
 
 ' Returns an array with the file names that match Path.
 ' The Path string may contain the wildcard characters "*"
 ' and "?" in the file name component. The same rules apply
 ' as with the MSDOS DIR command.
 ' If Path is a directory, the contents of this directory is listed.
 ' If Path is empty, the current directory is listed.
 ' Author: Christian d'Heureuse (www.source-code.biz)
 ' License: GNU/LGPL (http://www.gnu.org/licenses/lgpl.html)
 Public Function ListDir (ByVal Path)
    Dim fso: Set fso = CreateObject("Scripting.FileSystemObject")
    If Path = "" then Path = "*.*"
    Dim Parent, Filter
    if fso.FolderExists(Path) then      ' Path is a directory
       Parent = Path
       Filter = "*"
      Else
       Parent = fso.GetParentFolderName(Path)
       If Parent = "" Then If Right(Path,1) = ":" Then Parent = Path: Else Parent = "."
       Filter = fso.GetFileName(Path)
       If Filter = "" Then Filter = "*"
       End If
    ReDim a(10)
    Dim n: n = 0
    Dim Folder: Set Folder = fso.GetFolder(Parent)
    Dim Files: Set Files = Folder.Files
    Dim File
    For Each File In Files
       If CompareFileName(File.Name,Filter) Then
          If n > UBound(a) Then ReDim Preserve a(n*2)
          a(n) = File.Path
          n = n + 1
          End If
       Next
    ReDim Preserve a(n-1)
    ListDir = a
    End Function
 
 Private Function CompareFileName (ByVal Name, ByVal Filter) ' (recursive)
    CompareFileName = False
    Dim np, fp: np = 1: fp = 1
    Do
       If fp > Len(Filter) Then CompareFileName = np > len(name): Exit Function
       If Mid(Filter,fp) = ".*" Then    ' special case: ".*" at end of filter
          If np > Len(Name) Then CompareFileName = True: Exit Function
          End If
       If Mid(Filter,fp) = "." Then     ' special case: "." at end of filter
          CompareFileName = np > Len(Name): Exit Function
          End If
       Dim fc: fc = Mid(Filter,fp,1): fp = fp + 1
       Select Case fc
          Case "*"
             CompareFileName = CompareFileName2(name,np,filter,fp)
             Exit Function
          Case "?"
             If np <= Len(Name) And Mid(Name,np,1) <> "." Then np = np + 1
          Case Else
             If np > Len(Name) Then Exit Function
             Dim nc: nc = Mid(Name,np,1): np = np + 1
             If Strcomp(fc,nc,vbTextCompare)<>0 Then Exit Function
          End Select
       Loop
    End Function
 
 Private Function CompareFileName2 (ByVal Name, ByVal np0, ByVal Filter, ByVal fp0)
    Dim fp: fp = fp0
    Dim fc2
    Do                                  ' skip over "*" and "?" characters in filter
       If fp > Len(Filter) Then CompareFileName2 = True: Exit Function
       fc2 = Mid(Filter,fp,1): fp = fp + 1
       If fc2 <> "*" And fc2 <> "?" Then Exit Do
       Loop
    If fc2 = "." Then
       If Mid(Filter,fp) = "*" Then     ' special case: ".*" at end of filter
          CompareFileName2 = True: Exit Function
          End If
       If fp > Len(Filter) Then         ' special case: "." at end of filter
          CompareFileName2 = InStr(np0,Name,".") = 0: Exit Function
          End If
       End If
    Dim np
    For np = np0 To Len(Name)
       Dim nc: nc = Mid(Name,np,1)
       If StrComp(fc2,nc,vbTextCompare)=0 Then
          If CompareFileName(Mid(Name,np+1),Mid(Filter,fp)) Then
             CompareFileName2 = True: Exit Function
             End If
          End If
       Next
    CompareFileName2 = False
    End Function
 'End contributed functions by Christian d'Heureuse (www.source-code.biz)
 
 If arguments.Count < 2 Then
    usage()
 	WScript.Quit 1
 End If
 
 'initialize variables
 POINTS =  10
 RANSAC =   1
 CLEAN  =   0
 SIZE   = 800
 
 'maximum number of files that can be processed
 Dim InFiles(255)
 
 'status tells us where we are around the -o / --output option in parsing
 status =   0
 
 For i = 0 to arguments.length - 1
    If status = 1 Then
       'input files to be processed
       InFiles(j)=arguments(i)
       j = j + 1
       ' no echo like in original shell script
    Else
       'parse command switches, case unsensitive
       Select Case LCase(arguments(i))
          Case "-o", "--output"
             i=i+1
             PANOFILE=arguments(i)
             status = 1
             j = 0
          Case "-s", "--size"
             i=i+1
             SIZE=arguments(i)
          Case "-p", "--points"
             i=i+1
             POINTS=arguments(i)
          Case "-n", "--noransac"
             RANSAC = 0
          Case "-c", "--clean"
             CLEAN = 1
          Case "-h", "--help"
             usage()
          Case Else
             WScript.Echo "Command line parsing error at: " & arguments(i)
             WScript.Quit 1
       End Select
 ' NOTE: could make the script more robust by checking here
 ' if the first character of arguments(i+) is a -
 ' if it is not, set status to 1, this way the -o argument
 ' must not be the last of the arguments
    End If
 Next
 
 ' shell object, required to run executable
 Set objShell = CreateObject( "WScript.Shell" )
 ' allow user to override temporary folder - not implemented for now
 ' tmp=objShell.ExpandEnvironmentStrings("%TMP%")
 ' temp=objShell.ExpandEnvironmentStrings("%TEMP%")
 
 ' Filesystem object to play with filenames
 Set objFS = CreateObject("Scripting.FileSystemObject")
 
 ' set paths to autopano-sift-c executables or die
 ' first try the directory where this script was found
 exedir = objFS.GetParentFolderName( WScript.ScriptFullName )
 if not objFS.FileExists( exedir & "\" & autopano ) then
   ' next look in the standard place
   exedir = stdinstalldir
   if not objFS.FileExists( exedir & "\" & autopano ) then 
   ' finally try the the current dir
     exedir = "."
    if not objFS.FileExists( exedir & "\" & autopano ) then
      Wscript.echo "ERROR can't find autopano program"
      Wscript.Quit(3) 
    end if
   end if
 end if
 ' full pathnames, in quotes 
 generatekeys = """" & exedir & "\" & generatekeys & """"
 autopano = """" & exedir & "\" & autopano & """"
 
 KEYFILES = ""
 
 'if only one filename, assume it might be a wildcard and expand
 Dim ExpFiles
 if (1 = j) Then
 	ExpFiles = ListDir(InFiles(0))
 	j = UBound(ExpFiles)+1
 Else
   ExpFiles = InFiles
 End If
 
 Dim cmdLine
 Dim cmdResult
 Dim keyResult
 objShell.Popup "Please wait while keys are generated...", 2, "autopano-c-complete",64
 
 keyResult = "Keyfile generation results:" & chr(13)
 
 For i=0 To j-1
    ' get the absolute path to the image file
    ' and append the extension key to it
    ' create quoted versions of both the image and key files
    IMGFILE= objFS.GetAbsolutePathName(ExpFiles(i))
    IMGFILENAME= CHR(34) & IMGFILE & CHR(34)
    KEYFILE= IMGFILE & ".key"
    FILENAME= CHR(34) & KEYFILE & CHR(34)
    ' add the filename to the list of keyfiles
    KEYFILES = KEYFILES & " " & FILENAME
    cmdLine = generatekeys & " " & IMGFILENAME & " " & FILENAME & " " & SIZE
    ' if the file exists
    If objFS.FileExists(KEYFILE) Then
       if CLEAN <> 0 Then
 '         WScript.Echo "Regenerate using: " & cmdLine
 				  cmdResult = objShell.Run( cmdLine, 0, True )
 '         Set objExec=objShell.Exec("generatekeys " & IMGFILENAME & " "& FILENAME & " " & SIZE)
 '         Do Until objExec.Status = 1
 '            WScript.Sleep 100
 '         Loop
       Else
          cmdResult = 775577
       End If
    Else
       ' kick off generatekeys
 '   WScript.Echo "Create keyfile using: " & cmdLine
 	    cmdResult = objShell.Run( cmdLine, 0, True )
 '     WScript.Echo "generatekeys " & IMGFILENAME & " "& FILENAME & " " & SIZE
 '      Set objExec=objShell.Exec( "generatekeys " & IMGFILENAME & " "& FILENAME & " " & SIZE)
 '      Do Until objExec.Status = 1
 '         WScript.Sleep 100
 '      Loop
    End If
    If (0 <> cmdResult) Then
    	 if (775577 = cmdResult) Then
         keyResult = keyResult & chr(13) & FILENAME & " Reused keypoint file  "
      Else
         keyResult = keyResult & chr(13) & FILENAME & " Generatekeys failed?! Error " & cmdResult
      End If
    Else
      keyResult = keyResult & chr(13) & FILENAME & " OK  "
    End If
 Next
 
 ' Set timeout from 4 to 0 to wait forever for user to click
 objShell.Popup keyResult, 4, "autopano-c-complete",64
 ' WScript.Echo "keyfiles: "+KEYFILES
 ARG="--ransac " & RANSAC & " --maxmatches " & POINTS
 'WScript.Echo "autopano " & ARG & " " & PANOFILE & " " & KEYFILES
 objShell.Popup "Running autopano against key files", 2, "autopano-c-complete",64
 Set objExec=objShell.Exec( autopano & " " & ARG & " " & PANOFILE & " " & KEYFILES)
 Do Until objExec.Status = 1
    WScript.Sleep 100
 Loop