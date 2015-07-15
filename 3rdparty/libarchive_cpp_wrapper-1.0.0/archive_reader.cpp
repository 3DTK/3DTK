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

#include "archive_reader.hpp"

#include "archive_exception.hpp"
#include <archive_entry.h>

namespace ns_archive {

reader::reader(std::istream& stream, size_t block_size) :
  _archive( archive_read_new(), [](archive* archive){ archive_read_free(archive); } ), // errors in destructor will be silently ignored
  _buffer( new ns_reader::entry_buffer( _archive.get() ) ),
  _reader_container( stream, block_size )
{
  //
}

std::shared_ptr<entry> reader::get_next_entry()
{
  if(!has_next_entry())
  {
    throw archive_exception( "get_next_entry was called after all the entries were read" );
  }

  std::shared_ptr<entry> a_entry( _next_entry );
  _next_entry = nullptr;

  return a_entry;
}

bool reader::has_next_entry()
{
  bool has_next = true;

  if(_next_entry == nullptr)
  {
    archive_entry* a_entry;
    has_next = (archive_read_next_header(_archive.get(), &a_entry) == ARCHIVE_OK);

    if(has_next)
    {
      _next_entry = std::make_shared<entry>( a_entry, *_buffer.get() );
    }
  }

  return has_next;
}

/// ---------------- init_format ---------------- //

#define READER_INIT_FORMAT(__FORMAT__, __FUNCTION_SUFFIX__) \
template<> \
void reader::init_format<ns_reader::format::_##__FORMAT__>()\
{\
  archive_read_support_format_##__FUNCTION_SUFFIX__(_archive.get());\
}

READER_INIT_FORMAT(RAW, raw)
READER_INIT_FORMAT(ALL, all)
READER_INIT_FORMAT(7ZIP, 7zip)
READER_INIT_FORMAT(AR, ar)
READER_INIT_FORMAT(CAB, cab)
READER_INIT_FORMAT(CPIO, cpio)
READER_INIT_FORMAT(ISO9660, iso9660)
READER_INIT_FORMAT(LHA, lha)
READER_INIT_FORMAT(MTREE, mtree)
READER_INIT_FORMAT(RAR, rar)
READER_INIT_FORMAT(TAR, tar)
READER_INIT_FORMAT(XAR, xar)
READER_INIT_FORMAT(ZIP, zip)

/// ---------------- init_filter ---------------- //

#define READER_INIT_FILTER(__FILTER__, __FUNCTION_SUFFIX__) \
template<> \
void reader::init_filter<ns_reader::filter::_##__FILTER__>()\
{\
  archive_read_support_filter_##__FUNCTION_SUFFIX__(_archive.get());\
}

READER_INIT_FILTER(ALL, all)
READER_INIT_FILTER(BZIP2, bzip2)
READER_INIT_FILTER(COMPRESS, compress)
READER_INIT_FILTER(GZIP, gzip)
READER_INIT_FILTER(LZIP, lzip)
READER_INIT_FILTER(LZMA, lzma)
READER_INIT_FILTER(XZ, xz)
READER_INIT_FILTER(UU, uu)
READER_INIT_FILTER(RPM, rpm)
READER_INIT_FILTER(LRZIP, lrzip)
READER_INIT_FILTER(LZOP, lzop)
READER_INIT_FILTER(GRZIP, gzip)

/// ---------------- init_data ---------------- //
namespace ns_reader {

ssize_t reader_callback( archive* archive, void* in_reader_container, const void** buff )
{
  reader::reader_container* p_reader_container = reinterpret_cast<reader::reader_container*>( in_reader_container );

  p_reader_container->_stream.read( &p_reader_container->_buff[0], p_reader_container->_buff.size() );
  *buff = &p_reader_container->_buff[0];

  return p_reader_container->_stream.gcount();
}

} // ns_reader

void reader::init_data()
{
  if(archive_read_open( _archive.get(), &_reader_container, nullptr, ns_reader::reader_callback, nullptr ) != ARCHIVE_OK)
  {
    throw archive_exception( "Failed to read the archive!" );
  }
}

ns_reader::iterator reader::begin()
{
    return ns_reader::iterator( this, false );
}

ns_reader::iterator reader::end()
{
    return ns_reader::iterator( this, true );
}

}
