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

#ifndef ARCHIVE_WRITER_HPP_INCLUDED
#define ARCHIVE_WRITER_HPP_INCLUDED

#include <ostream>
#include <memory>
#include <archive.h>

#include "archive_writer_format.hpp"
#include "archive_writer_filter.hpp"
#include "archive_entry.hpp"

namespace ns_archive {

namespace ns_writer {

ssize_t writer_callback( archive* archive, void* out_writer_container, const void* buff, size_t buff_size );

} // ns_writer

class writer
{
public:
  writer(writer&&) = default;

  writer(const writer&) = delete;
  writer& operator=(const writer&) = delete;

  void finish();

  template<ns_writer::format FORMAT, ns_writer::filter FILTER>
  static writer make_writer(std::ostream& stream, size_t block_size);

  template<ns_writer::format FORMAT>
  static writer make_writer(std::ostream& stream, size_t block_size);

  void add_entry( entry& a_entry );

private:
  writer(std::ostream& stream, size_t block_size);

  template<ns_writer::format FORMAT>
  void init_format();

  template<ns_writer::filter FILTER>
  void init_filter();

  void init_data();

  std::shared_ptr<archive> _archive;
  std::ostream& _writer_container;
  size_t _block_size;

  friend ssize_t ns_writer::writer_callback( archive* archive, void* out_writer_container, const void* buff, size_t buff_size );
};

}

#include "archive_writer.ipp"

#endif // ARCHIVE_WRITER_HPP_INCLUDED

