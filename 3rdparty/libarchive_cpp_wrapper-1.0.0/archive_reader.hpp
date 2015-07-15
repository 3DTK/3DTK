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

#ifndef ARCHIVE_READER_HPP_INCLUDED
#define ARCHIVE_READER_HPP_INCLUDED

#include <istream>
#include <memory>
#include <vector>
#include <archive.h>

#include "archive_reader_format.hpp"
#include "archive_reader_filter.hpp"
#include "archive_entry.hpp"
#include "archive_reader_iterator.hpp"

namespace ns_archive {
namespace ns_reader {

ssize_t reader_callback( archive* archive, void* in_reader_container, const void** buff );

} // ns_reader

class reader
{
public:
  reader(reader&&) = default;

  reader(const reader&) = delete;
  reader& operator=(const reader&) = delete;

  template<ns_reader::format FORMAT>
  static reader make_reader( std::istream& stream, size_t block_size );

  template<ns_reader::format FORMAT, ns_reader::filter FILTER>
  static reader make_reader( std::istream& stream, size_t block_size);

  template<ns_reader::filter FILTER>
  static reader make_reader( std::istream& stream, size_t block_size);

  std::shared_ptr<entry> get_next_entry();
  bool has_next_entry();

  ns_reader::iterator begin();
  ns_reader::iterator end();

private:
  reader( std::istream& stream, size_t block_size );

  template<ns_reader::format FORMAT>
  void init_format();

  template<ns_reader::filter FILTER>
  void init_filter();

  void init_data();

  std::shared_ptr<archive> _archive;
  std::shared_ptr<entry> _next_entry = nullptr;
  std::shared_ptr<ns_reader::entry_buffer> _buffer;

  class reader_container
  {
  public:
    reader_container( std::istream& stream, size_t block_size ) :
      _stream( stream )
    {
      _buff.resize( block_size );
    }
  public:
    std::istream& _stream;
    std::vector<char> _buff;
  } _reader_container;

  friend ssize_t ns_reader::reader_callback( archive* archive, void* in_reader_container, const void** buff );
};

}

#include "archive_reader.ipp"

#endif // ARCHIVE_READER_HPP_INCLUDED
