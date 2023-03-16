/*

  embroidery.h - plugin for reading and executing embroidery file from SD card.

  Copyright (c) 2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _EMBROIDERY_H_
#define _EMBROIDERY_H_

#include "driver.h"

typedef enum {
    Stitch_Normal,
    Stitch_Trim,
    Stitch_Jump,
    Stitch_Stop,
    Stitch_SequinEject
} stich_type_t;

typedef uint8_t embroidery_thread_color_t;

typedef struct {
    stich_type_t type;
    embroidery_thread_color_t color;
    coord_data_t target;
} stitch_t;

typedef bool (*get_stitch_ptr)(stitch_t *stitch, vfs_file_t *file);
typedef const char *(*get_thread_color_ptr)(uint8_t color);
typedef void (*thread_trim_ptr)(void);
typedef void (*thread_change_ptr)(embroidery_thread_color_t color);

typedef struct {
    get_stitch_ptr get_stitch;
    get_thread_color_ptr get_thread_color;
    thread_trim_ptr thread_trim;
    thread_change_ptr thread_change;
    const char *name;
    uint32_t stitches;
    uint32_t threads;
    uint32_t trims;
    uint32_t color_changes;
    coord_data_t min;
    coord_data_t max;
    coord_data_t size;
} embroidery_t;

typedef bool (*open_file_ptr)(vfs_file_t *file, embroidery_t *api);

const char *embroidery_get_thread_color (embroidery_thread_color_t color);
void embroidery_set_thread_trim_handler (thread_trim_ptr handler);
void embroidery_set_thread_change_handler (thread_change_ptr handler);

#endif // _EMBROIDERY_H_
