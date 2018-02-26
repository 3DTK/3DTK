This file describes the scanserver functionality, the code changes and its behaviour.

To run with the scanserver functionality, pass -S or --scanserver to the slam6D/show call. Start the scanserver with bin/scanserver & first. If you want to directly jump to usage examples, see the "USAGE" section below.

The scanserver is a new method to load and manage scans for 'slam6D', 'show' and some few other tools (so far). It removes all the IO code from the clients and handles it in the server process. This separation offers persistence of scan data and avoids unneccessary reloads of full scans or even reduced versions thereof. By using a caching framework it also transparently handles the available memory given and enables (nearly) endless amounts of data. The client is only required to open the interface, load a directory and start working on those scans without having to alter its workflow (e.g., pre-reduce them) to accomodate huge data volumes.

If you have questions or problems (or both), contact Thomas Escher <tescher@uos.de>.



USAGE:

1. General

Start the scanserver once (in another terminal, or in the same one as a background process):
  bin/scanserver &
  
Do all the normal work as you would normally do, adding the parameter -S:
  bin/slam6D dat -S
  bin/show dat -S

2. Changing the available memory size

Changing the cache memory size used by scan data (about half the system memory usually works):

  bin/scanserver -c 3500   (for 8GB RAM)

If you intend to not reload the full scans for different reduction parameters or don't have too much memory/disk space, disable binary scan caching. Binary scan caching saves the full scans as long as the range or height parameters aren't touched, which would cause a full reload:

  bin/scanserver -b 0

If your dataset contains many scans and loops (e.g., 'hannover' with 468 scans), the default data memory (150M) won't be enough to hold all the animation frames and you need to increase it:

  bin/scanserver -d 250

3. Altering the shared memory on your linux system (bus_error)

If you receive a bus_error, the size of your shared memory is too small and the requested allocation was too big. This is resolved via remounting your shm device. Default is half of the available RAM. This limit can be increased to nearly 90% of the RAM if required.

  sudo mount -o remount,size=7000M /dev/shm   (for 8GB RAM)

4. Locking the memory to avoid swapping (Linux only)

If a great portion of the RAM is used for the cache data, swapping will usually occur after 50% of usage. To avoid this, the scanserver tries to lock the whole memory in place. This will fail without superuser rights, as well as on a full shared memory (see 3.) even with rights. To solve this problem, add these two additional lines to '/etc/security/limits.conf':

  * soft memlock unlimited
  * hard memlock unlimited
After adding these lines and rebooting the system the scanserver can be started without superuser rights. 

5. Using the octtree serialization feature in show with scanserver

The octtree serialization behaves slightly different than before. Since the scanserver caches octtrees between calls of 'show', the loading of octtrees only becomes relevant if no octtrees are in the cache and have to be created from the scan itself. If this has been done once before and the octtrees have been saved via --saveOct before this can be used to speed up the octtree loading with --loadOct.

  bin/show dat --loadOct --saveOct

If octtrees are not cached, they are deserialized if available, created otherwise and then saved for future calls.

IMPLEMENTATION STATUS:

Currently only 'slam6D' and 'show' are working with the scanserver.

Since the scanserver handles the disk IO now and the filtering has been optimized, not all ScanIOs are updated yet. Just copy and paste and change the minor parts about reading the input lines.

Working: 'ls src/scanio' in a shell
