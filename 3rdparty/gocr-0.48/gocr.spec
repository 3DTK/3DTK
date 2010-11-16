%define name	gocr
%define version	0.48
%define release	1

%define prefix		%{_prefix}
%define _xprefix	/usr
%define _xbindir	%{_xprefix}/bin

Summary:	Gocr is an OCR (Optical Character Recognition) program.
Name:		%{name}
Version:	%{version}
Release:	%{release}
URL:		http://jOCR.sourceforge.net/
License:	GPL
Group:		Graphics
Source0:	http://prdownloads.sourceforge.net/jocr/%{name}-%{version}.tar.gz
#Source1:	%{name}-16x16.xpm.bz2
#Source2:	%{name}-32x32.xpm.bz2
#Patch0:         %{name}-DESTDIR.patch.bz2
#Patch1:         %{name}-paths.patch.bz2
#following packages are not really needed, but recommended
#BuildRequires:  libnetpbm1-devel
#BuildRequires:  gtk+-devel >= 1.2.8
#BuildRequires:  transfig
# set RPM_BUILD_ROOT to /var/tmp/gocr-buildroot , overwritten by .rpmmacros
BuildRoot:	%{_tmppath}/%{name}-buildroot

%description
GOCR is an optical character recognition program. 
It reads images in many formats  and outputs a text file.
Possible image formats are pnm, pbm, pgm, ppm, some pcx and
tga image files. Other formats like pnm.gz, pnm.bz2, png, jpg, tiff, gif,
bmp will be automatically converted using the netpbm-progs, gzip and bzip2
via unix pipe.
A simple graphical frontend written in tcl/tk and some
sample files (you need transfig for the sample files) are included.
Gocr is also able to recognize and translate barcodes.
You do not have to train the program or store large font bases.
Simply call gocr from the command line and get your results.


#%package -n %{name}-devel
#Summary:	Development tools for gocr.
#Group:		Development/C

#%description -n %{name}-devel
#GOCR is an optical character recognition program.
#If you want to develop programs which will manipulate gocr, you should 
#install gocr-devel.  You'll also need to install the gocr package.


# not tested, what about gocr.tcl ???
# switched off because gtk-devel was not available on my PC (joerg Aug06)
# and you dont need it to get gocr working
#%package -n %{name}-gtk
#Summary:	Gtk+ frontend for gocr
#Group: 		Graphics
#Requires:	%{name} = %{version}
#BuildRequires:  gtk-devel >= 1.2.8

#% description -n %{name}-gtk
#Gtk-gocr is a graphical frontend to GOCR the
#optical character recognition program.
# 
#Gtk+-based frontend for gocr.

# rpmbuild -bp: unpacking sources and applaying patches
%prep
rm -rf $RPM_BUILD_ROOT

%setup -q
#%patch0 -p1
#%patch1 -p1

# rpmbuild -bc: build, after prep, invokes make
# please help
%build
%configure --with-netpbm=no
%{__make} DESTDIR=$RPM_BUILD_ROOT
echo "build end"

# [ -e config.cache ] && rm -f config.cache
# % configure --with-netpbm=no --prefix=%{_xprefix} --bindir=%{_xbindir}
# % {__make}

# rpmbuild -bi: install-stage
%install

%{__make} install DESTDIR=$RPM_BUILD_ROOT

# not needed anymore
# % post -n %name-gtk
# % {update_menus}
#   
# % postun -n %name-gtk
# % {clean_menus}
echo "install end"
   
%clean
rm -rf $RPM_BUILD_ROOT
echo "clean end"

# rpmbuild -bl: list check, check that every file exists
%files -n %{name}
%defattr(-, root, root)
%doc AUTHORS BUGS CREDITS HISTORY README REMARK.txt TODO
%doc doc/{examples.txt,gocr.html,unicode.txt}
%doc examples/{ex.fig,font2.fig,ocr-a.png,ocr-b.png}
%{_mandir}/man1/*
%attr(755,root,root) %{_bindir}/*

#%files -n %{name}-devel
#%defattr(-, root, root)
#%doc AUTHORS REVIEW
#%{_libdir}/libPgm2asc.a
#%{_includedir}/gocr.h

#%files -n %{name}-gtk
#%defattr(-, root, root)
#%doc frontend/gnome/{AUTHORS,README,TODO}
##%{_menudir}/*
##%{_iconsdir}/*
#%attr(755,root,root) %{_xbindir}/*

%changelog
* Tue Feb  5 2008 Joerg Schulenburg, 0.46
- remove devel part (for simplicity)

* Wed Aug 30 2006 Joerg Schulenburg, 0.41
- remove gtk-part (dont work on my PC)

* Sat Dec 27 2003 Nick Urbanik <nicku(at)vtc.edu.hk> 0.38-1.1nu
- Change %make to %{__make}
- check config.cache exists before trying to delete it
- Fix the files list for documentation.
- add back the gtk frontend
- Added the correct build require for gtk frontend
- various other fixes to get it to build.

* Tue Jan 29 2001 Joerg Schulenburg <jschulen-at-gmx.de-NOSPAM> 0.3.6-1
- gocr.spec taken krom  Marcel Pol <mpol(at)gmx.net>

# check: rpmbuild -ba --nobuild gocr.spec
