/*
 * Copyright 2019 NXP
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

#ifndef __AF_WATCHDOG_H
#define __AF_WATCHDOG_H

#define WD_MSG_TYPE_ALIVE   (1 << 0)
#define WD_MSG_TYPE_TIMEOUT (1 << 1)
#define WD_MSG_TYPE_WARN_MEM    (1 << 2)
#define WD_MSG_TYPE_WARN_CPU    (1 << 3)
#define WD_MSG_TYPE_WARN_AF (1 << 4)

#define WD_MAGIC (0x65708768) /* "afwd" */

struct watchdog_message
{
    uint32_t magic;
    uint16_t type;
    uint16_t subtype;
};

#endif /* ! __AF_WATCHDOG_H */
