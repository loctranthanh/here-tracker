/**************************************************************************************************
 * Copyright (C) 2017-2018 HERE Europe B.V.                                                       *
 * All rights reserved.                                                                           *
 *                                                                                                *
 * MIT License                                                                                    *
 * Permission is hereby granted, free of charge, to any person obtaining a copy                   *
 * of this software and associated documentation files (the "Software"), to deal                  *
 * in the Software without restriction, including without limitation the rights                   *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell                      *
 * copies of the Software, and to permit persons to whom the Software is                          *
 * furnished to do so, subject to the following conditions:                                       *
 *                                                                                                *
 * The above copyright notice and this permission notice shall be included in all                 *
 * copies or substantial portions of the Software.                                                *
 *                                                                                                *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR                     *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                       *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE                    *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                         *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,                  *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE                  *
 * SOFTWARE.                                                                                      *
 **************************************************************************************************/

#ifndef HERE_TRACKING_TLS_WRITER_H
#define HERE_TRACKING_TLS_WRITER_H

#include <stdint.h>
#include <string.h>

#include "here_tracking_data_buffer.h"
#include "here_tracking_error.h"
#include "here_tracking_tls.h"

typedef struct
{
    here_tracking_tls tls_ctx;
    here_tracking_data_buffer data_buffer;
} here_tracking_tls_writer;

here_tracking_error here_tracking_tls_writer_init(here_tracking_tls_writer* writer,
                                                  here_tracking_tls tls_ctx,
                                                  uint8_t* write_buf,
                                                  size_t write_buf_size);

here_tracking_error here_tracking_tls_writer_write_char(here_tracking_tls_writer* writer,
                                                        char c);

here_tracking_error here_tracking_tls_writer_write_data(here_tracking_tls_writer* writer,
                                                        const uint8_t* data,
                                                        size_t data_size);

here_tracking_error here_tracking_tls_writer_write_string(here_tracking_tls_writer* writer,
                                                          const char* s);

here_tracking_error here_tracking_tls_writer_write_utoa(here_tracking_tls_writer* writer,
                                                        uint32_t u,
                                                        uint8_t base);

here_tracking_error here_tracking_tls_writer_flush(here_tracking_tls_writer*);

#endif /* HERE_TRACKING_TLS_WRITER_H */
