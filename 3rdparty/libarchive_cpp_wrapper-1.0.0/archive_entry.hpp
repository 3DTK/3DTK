/*
BSD 2-Clause license

Copyright (c) 2014, Domen Vrankar
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ARCHIVE_WRITER_ENTRY_HPP_INCLUDED
#define ARCHIVE_WRITER_ENTRY_HPP_INCLUDED

#include <archive.h>
#include <archive_entry.h>
#include <string>
#include <cstdint>
#include <istream>
#include <memory>
#include <linux/types.h>

#include "archive_reader_entry_buffer.hpp"

namespace ns_archive {

class entry
{
public:
  entry(archive_entry* entry, ns_reader::entry_buffer& entry_buffer); // TODO change to private and friend reader?
  entry(std::istream& stream);

  void clear_entry(std::istream& stream);

  int64_t get_header_value_gid();
  int64_t get_header_value_ino();
  int64_t get_header_value_ino64();
  int64_t get_header_value_size();
  int64_t get_header_value_uid();
  mode_t get_header_value_mode();
  mode_t get_header_value_perm();
  dev_t get_header_value_rdev();
  dev_t get_header_value_rdevmajor();
  dev_t get_header_value_rdevminor();
  std::string get_header_value_gname();
  std::string get_header_value_hardlink();
  std::string get_header_value_pathname();
  std::string get_header_value_symlink();
  std::string get_header_value_uname();
  unsigned int get_header_value_nlink();


  void set_header_value_gid(int64_t value);
  void set_header_value_ino(int64_t value);
  void set_header_value_ino64(int64_t value);
  void set_header_value_size(int64_t value);
  void set_header_value_uid(int64_t value);
  void set_header_value_mode(mode_t value);
  void set_header_value_perm(mode_t value);
  void set_header_value_rdev(dev_t value);
  void set_header_value_rdevmajor(dev_t value);
  void set_header_value_rdevminor(dev_t value);
  void set_header_value_gname(const std::string& value);
  void set_header_value_hardlink(const std::string& value);
  void set_header_value_link(const std::string& value);
  void set_header_value_pathname(const std::string& value);
  void set_header_value_symlink(const std::string& value);
  void set_header_value_uname(const std::string& value);
  void set_header_value_nlink(unsigned int value);
  void set_header_value_mtime(time_t time, long value);

  archive_entry* get_entry() const;
  std::istream& get_stream();

private:
  std::shared_ptr<archive_entry> _entry;
  std::istream _stream;
  bool _has_stream;
};

}

#endif // ARCHIVE_WRITER_ENTRY_HPP_INCLUDED
