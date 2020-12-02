/*
 * Copyright 2019-2020 NXP
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __CIPC_H
#define __CIPC_H
#include <stddef.h>

/**
 * File management flags
 *
 * FILE_O_CREATE - Create a new file, delete previous file with same name if any
 * FILE_O_APPEND - Append to existing file if exists, or create a new one
 */
#define FILE_O_CREATE     0x01
#define FILE_O_APPEND     0x02


/**
 * @brief Copy available data from one ivshm binary path
 *
 * @param[in] id Endpoint id of the ivshm binary path
 * @param[in] buf Destination buffer where to copy the data to
 * @param[in] len The maximum number of bytes to read
 *
 * @return The actual number of bytes which were read, or negative value if error
 */
ssize_t cipc_read_buf(unsigned id, void *buf, size_t len);

/**
 * @brief Copy available data from one ivshm binary path
 *
 * @param[in] id    Endpoint id of the ivshm binary path
 * @param[in] buf   Destination buffer where to copy the data to
 * @param[in] len   The maximum number of bytes to read
 *
 * @return The actual number of bytes which were read, or negative value
 *  if error
 */
ssize_t cipc_read_buf_blocking(unsigned id, void *buf, size_t len);

/**
 * @brief Copy data from a file of the second OS into a buffer, through one
 * ivshm binary path. A daemon running on the second OS must open the file
 * and fill the buffer.
 *
 * @param[in] id    Endpoint id of the ivshm binary path
 * @param[in] buf   Source buffer where to copy the data
 * @param[in] len   Number of bytes to be read: if 0 all the file is read
 * @param[in] name  The name of the file to be read
 *
 * @return The actual number of read bytes, or negative value if error
 */
ssize_t cipc_read_file(unsigned id, void *buf, size_t len, char *name);

/**
 * @brief Reports the size of a file of the second OS into a buffer, through
 * one ivshm binary path. A daemon running on the second OS must open the file
 * and get file size.
 *
 * @param[in] id    Endpoint id of the ivshm binary path
 * @param[in] name  The name of the file
 *
 * @return The actual file size in bytes, or negative value if error
 */
ssize_t cipc_size_file(unsigned id, char *name);

/**
 * @brief Copy data from buffer into one ivshm binary path, so it can be
 * accessed by second OS
 *
 * @param[in] id Endpoint id of the ivshm binary path
 * @param[in] buf Source buffer where to copy the data from
 * @param[in] len Number of bytes to copy
 *
 * @return The actual number of bytes which were written, or negative value if error
 */
ssize_t cipc_write_buf(unsigned id, void *buf, size_t len);

/**
 * @brief Copy data from buffer into a file of the second OS, through one ivshm
 * binary path. An application running on the second OS must be in charge of file
 * creation and filling
 *
 * @param[in] id Endpoint id of the ivshm binary path
 * @param[in] buf Source buffer where to copy the data from
 * @param[in] len Number of bytes to copy
 * @param[in] name File name where the data are copied to
 * @param[in] oflags File management flag (see FILE_O_* above)
 *
 * @return The actual number of bytes which were written, or negative value if error
 */
ssize_t cipc_write_file(unsigned id, void *buf, size_t len,
                       char *name, uint32_t oflags);

#endif
