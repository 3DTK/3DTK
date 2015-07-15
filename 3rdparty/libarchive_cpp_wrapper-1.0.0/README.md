libarchive cpp wrapper library
============================
This is a C++ wrapper arround libarchive library.

Dependencies
============================
- libarchive (https://github.com/libarchive/libarchive)
- CMake (http://cmake.org/)

Current version was tested with gcc 4.9 on 64 bit Linux - Ubuntu 14.04 with libarchive13 3.1.2-9

Example usage
============================

Read from archive

```C++
#include "archive_reader.hpp"
#include "archive_exception.hpp"

...

try
{
  namespace ar = ns_archive::ns_reader;
  std::fstream fs("some_tar_file.tar");
  ns_archive::reader reader = ns_archive::reader::make_reader<ar::format::_ALL, ar::filter::_ALL>(fs, 10240);

  for(auto entry : reader)
  {
    // get file name
    std::cout << entry->get_header_value_pathname() << std::endl;
    // get file content
    std::cout << entry->get_stream().rdbuf() << std::endl << std::endl;
  }
}
catch(ns_archive::archive_exception& e)
{
  std::cout << e.what() << std::endl;
}
```

Write to archive

```C++
#include "archive_writer.hpp"
#include "archive_exception.hpp"

...

try
{
  namespace ar = ns_archive::ns_reader;
  std::ofstream outfs("output.tar");
  ns_archive::writer writer2 = ns_archive::writer::make_writer<ns_archive::ns_writer::format::_TAR>(outfs, 10240);
  std::stringstream ss;
  ss << "foo";

  ns_archive::entry out_entry(ss);
  out_entry.set_header_value_pathname("foo.txt");
  writer.add_entry(out_entry);
}
catch(ns_archive::archive_exception& e)
{
  std::cout << e.what() << std::endl;
}
```

Copy archive

```C++
#include "archive_reader.hpp"
#include "archive_writer.hpp"
#include "archive_exception.hpp"

...

try
{
  namespace ar = ns_archive::ns_reader;
  std::fstream fs( "some_tar_file.tar" );
  ns_archive::reader reader = ns_archive::reader::make_reader<ar::format::_ALL, ar::filter::_ALL>(fs, 10240);

  std::ofstream outfs( "out.tar" );
  ns_archive::writer writer = ns_archive::writer::make_writer<ns_archive::ns_writer::format::_TAR>(outfs, 10240);

  for(auto entry : reader)
  {
    writer.add_entry(*entry.get());
  }
}
catch(ns_archive::archive_exception& e)
{
  std::cout << e.what() << std::endl;
}
```

License
============================
BSD 2-Clause license (http://opensource.org/licenses/bsd-license.php)
