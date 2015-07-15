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

#include "archive_writer.hpp"

#include "archive_exception.hpp"
#include "string"

#include <vector>
#include <istream>

namespace ns_archive {

writer::writer(std::ostream& stream, size_t block_size) :
  _archive( archive_write_new(), [](archive* archive){ archive_write_free(archive); } ), // errors in destructor will be silently ignored
  _writer_container( stream ),
  _block_size( block_size )
{
  // TODO enable setting of padding parameters in constructor
  /// OTHER OPTION archive_write_set_bytes_per_block( _archive.get(), 0 ); // prevent zero padding of last block (this also causes write callback to be called on each write without blocking until a block is full)
  archive_write_set_bytes_in_last_block( _archive.get(), 1 ); // prevent zero padding of last block
}

void writer::finish()
{
  archive_write_close(_archive.get()); // TODO throw on error
}

void writer::add_entry( entry& a_entry )
{
  if( archive_write_header( _archive.get(), a_entry.get_entry() ) != ARCHIVE_OK)
  {
    throw archive_exception( "Error writing header to archive!" );
  }

  auto& stream = a_entry.get_stream();
  std::vector<char> buffer( _block_size );

  while( stream )
  {
    stream.read( &buffer[0], _block_size );
    archive_write_data( _archive.get(), &buffer[0], stream.gcount() );
  }
}

/// ---------------- init_format ---------------- //

#define WRITER_INIT_FORMAT(__FORMAT__, __FUNCTION_SUFFIX__) \
template<> \
void writer::init_format<ns_writer::format::_##__FORMAT__>()\
{\
  archive_write_set_format_##__FUNCTION_SUFFIX__(_archive.get());\
}

WRITER_INIT_FORMAT(7ZIP, 7zip)
WRITER_INIT_FORMAT(CPIO_SVR4_NOCRC, cpio_newc)
WRITER_INIT_FORMAT(ISO9660, iso9660)
WRITER_INIT_FORMAT(MTREE, mtree)
WRITER_INIT_FORMAT(SHAR, shar)
WRITER_INIT_FORMAT(SHAR_BASE, shar)
WRITER_INIT_FORMAT(SHAR_DUMP, shar_dump)
WRITER_INIT_FORMAT(TAR, pax_restricted)
WRITER_INIT_FORMAT(TAR_GNUTAR, gnutar)
WRITER_INIT_FORMAT(TAR_PAX_INTERCHANGE, pax)
WRITER_INIT_FORMAT(TAR_PAX_RESTRICTED, pax_restricted)
WRITER_INIT_FORMAT(TAR_USTAR, ustar)
WRITER_INIT_FORMAT(XAR, xar)
WRITER_INIT_FORMAT(ZIP, zip)

/// ---------------- init_filter ---------------- //

#define WRITER_INIT_FILTER(__FILTER__, __FUNCTION_SUFFIX__) \
template<> \
void writer::init_filter<ns_writer::filter::_##__FILTER__>()\
{\
  archive_write_add_filter_##__FUNCTION_SUFFIX__(_archive.get());\
}

WRITER_INIT_FILTER(NONE, none)
WRITER_INIT_FILTER(GZIP, gzip)
WRITER_INIT_FILTER(BZIP2, bzip2)
WRITER_INIT_FILTER(COMPRESS, compress)
WRITER_INIT_FILTER(GRZIP, grzip)
WRITER_INIT_FILTER(LRZIP, lrzip)
WRITER_INIT_FILTER(LZIP, lzip)
WRITER_INIT_FILTER(LZMA, lzma)
WRITER_INIT_FILTER(LZOP, lzip)
WRITER_INIT_FILTER(UU, uuencode)
WRITER_INIT_FILTER(XZ, xz)

/// ---------------- init_data ---------------- //
namespace ns_writer {

ssize_t writer_callback( archive* archive, void* out_writer_container, const void* buff, size_t buff_size )
{
  std::ostream* stream = reinterpret_cast<std::ostream*>( out_writer_container );

  stream->write( (const char*)buff, buff_size );

  return buff_size;
}

} // ns_writer

void writer::init_data()
{
  if(archive_write_open( _archive.get(), &_writer_container, nullptr, ns_writer::writer_callback, nullptr ) != ARCHIVE_OK)
  {
    throw archive_exception( "Failed to write the archive!" );
  }
}

}
