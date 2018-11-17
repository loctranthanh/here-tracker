/**************************************************************************************************
 * Copyright (C) 2017 HERE Europe B.V.                                                            *
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

#include <stddef.h>

#include "here_tracking_version.h"

/**************************************************************************************************/

uint16_t here_tracking_get_version_major()
{
    return HERE_TRACKING_VERSION_MAJOR;
}

/**************************************************************************************************/

uint16_t here_tracking_get_version_minor()
{
    return HERE_TRACKING_VERSION_MINOR;
}

/**************************************************************************************************/

uint16_t here_tracking_get_version_patch()
{
    return HERE_TRACKING_VERSION_PATCH;
}

/**************************************************************************************************/

void here_tracking_get_version(uint16_t* major, uint16_t* minor, uint16_t* patch)
{
    if(major != NULL)
    {
        *major = HERE_TRACKING_VERSION_MAJOR;
    }

    if(minor != NULL)
    {
        *minor = HERE_TRACKING_VERSION_MINOR;
    }

    if(patch != NULL)
    {
        *patch = HERE_TRACKING_VERSION_PATCH;
    }
}

/**************************************************************************************************/

const char* here_tracking_get_version_string()
{
    return HERE_TRACKING_VERSION_STRING;
}
