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

#include "archive_entry.hpp"
#include "archive_exception.hpp"

namespace ns_archive {

entry::entry(archive_entry* entry, ns_reader::entry_buffer& entry_buffer) :
  _entry( entry, []( archive_entry* entry ){ /* do nothing as this entry is owned by a reader */ } ),
  _stream(&entry_buffer),
  _has_stream(true)
{
  //
}

entry::entry(std::istream& stream) :
  _entry( archive_entry_new(), []( archive_entry* entry ){ archive_entry_free( entry ); } ),
  _stream( stream.rdbuf() ),
  _has_stream(true)
{
  auto* pbuf = _stream.rdbuf();
  std::size_t stream_size = pbuf->pubseekoff( 0, _stream.end, _stream.in );
  pbuf->pubseekpos( 0, _stream.in );

  archive_entry_set_mode( _entry.get(), S_IFREG );
  archive_entry_set_size( _entry.get(), stream_size );
}

void entry::clear_entry(std::istream& stream)
{
  archive_entry_clear( _entry.get() );
  _stream.rdbuf( stream.rdbuf() );
  _has_stream = true;

  auto* pbuf = _stream.rdbuf();
  std::size_t stream_size = pbuf->pubseekoff( 0, _stream.end, _stream.in );
  pbuf->pubseekpos( 0, _stream.in );

  archive_entry_set_mode( _entry.get(), S_IFREG );
  archive_entry_set_size( _entry.get(), stream_size );
}

//-------------------------getters---------------------------//
int64_t entry::get_header_value_gid()
{
  return archive_entry_gid(_entry.get());
}

int64_t entry::get_header_value_ino()
{
  return archive_entry_ino(_entry.get());
}

int64_t entry::get_header_value_ino64()
{
  return archive_entry_ino64(_entry.get());
}

int64_t entry::get_header_value_size()
{
  return archive_entry_size(_entry.get());
}

int64_t entry::get_header_value_uid()
{
  return archive_entry_uid(_entry.get());
}

mode_t entry::get_header_value_mode()
{
  return archive_entry_mode(_entry.get());
}

mode_t entry::get_header_value_perm()
{
  return archive_entry_perm(_entry.get());
}

dev_t entry::get_header_value_rdev()
{
  return archive_entry_rdev(_entry.get());
}

dev_t entry::get_header_value_rdevmajor()
{
  return archive_entry_rdevmajor(_entry.get());
}

dev_t entry::get_header_value_rdevminor()
{
  return archive_entry_rdevminor(_entry.get());
}

std::string entry::get_header_value_gname()
{
  return archive_entry_gname(_entry.get());
}

std::string entry::get_header_value_hardlink()
{
  return archive_entry_hardlink(_entry.get());
}

std::string entry::get_header_value_pathname()
{
  return archive_entry_pathname(_entry.get());
}

std::string entry::get_header_value_symlink()
{
  return archive_entry_symlink(_entry.get());
}

std::string entry::get_header_value_uname()
{
  return archive_entry_uname(_entry.get());
}

unsigned int entry::get_header_value_nlink()
{
  return archive_entry_nlink(_entry.get());
}

//-------------------------setters---------------------------//
void entry::set_header_value_gid(int64_t value)
{
  archive_entry_set_gid(_entry.get(), value);
}

void entry::set_header_value_ino(int64_t value)
{
  archive_entry_set_ino(_entry.get(), value);
}

void entry::set_header_value_ino64(int64_t value)
{
  archive_entry_set_ino64(_entry.get(), value);
}

void entry::set_header_value_size(int64_t value)
{
  archive_entry_set_size(_entry.get(), value);
}

void entry::set_header_value_uid(int64_t value)
{
  archive_entry_set_uid(_entry.get(), value);
}

void entry::set_header_value_mode(mode_t value)
{
  archive_entry_set_mode(_entry.get(), value);
}

void entry::set_header_value_perm(mode_t value)
{
  archive_entry_set_perm(_entry.get(), value);
}

void entry::set_header_value_rdev(dev_t value)
{
  archive_entry_set_rdev(_entry.get(), value);
}

void entry::set_header_value_rdevmajor(dev_t value)
{
  archive_entry_set_rdevmajor(_entry.get(), value);
}

void entry::set_header_value_rdevminor(dev_t value)
{
  archive_entry_set_rdevminor(_entry.get(), value);
}

void entry::set_header_value_gname(const std::string& value)
{
  archive_entry_set_gname(_entry.get(), value.c_str());
}

void entry::set_header_value_hardlink(const std::string& value)
{
  archive_entry_set_hardlink(_entry.get(), value.c_str());
}

void entry::set_header_value_link(const std::string& value)
{
  archive_entry_set_link(_entry.get(), value.c_str());
}

void entry::set_header_value_pathname(const std::string& value)
{
  archive_entry_set_pathname(_entry.get(), value.c_str());
}

void entry::set_header_value_symlink(const std::string& value)
{
  archive_entry_set_symlink(_entry.get(), value.c_str());
}

void entry::set_header_value_uname(const std::string& value)
{
  archive_entry_set_uname(_entry.get(), value.c_str());
}

void entry::set_header_value_nlink(unsigned int value)
{
  archive_entry_set_nlink(_entry.get(), value);
}

void entry::set_header_value_mtime(time_t time, long value)
{
  archive_entry_set_mtime(_entry.get(), time, value);
}

archive_entry* entry::get_entry() const
{
  return _entry.get();
}

std::istream& entry::get_stream()
{
  if(_has_stream)
  {
    _has_stream = false;

    return _stream;
  }

  throw archive_exception( "Archive entry stream was already read!" );
}

}
