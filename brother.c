/*

  brother.c - plugin for parsing Brother PAC data embedded in PAS format file read from SD card.

  Part of grblHAL

  Copyright (c) 2017 Fredrik Noring (palette)
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

#include "embroidery.h"

#if EMBROIDERY_ENABLE

typedef union {
    uint8_t buf[12];
    struct {
        char version[8];
        uint32_t pec_offset;
    };
} pes_header_t;

typedef union {
    uint8_t buf[512];
    struct {
        char label_prefix[3];
        char label[17];
        uint8_t unknown_1[11];
        uint8_t unknown_2;
        uint16_t unknown_3;
        uint8_t thumbnail_width;
        uint8_t thumbnail_height;
        uint8_t unknown_4[12];
        uint8_t n_colors;
        uint8_t palette_index[256];
    };
} pec_section1_t;

typedef union {
    uint8_t buf[36];
    struct {
        uint16_t unknown_1;
        uint16_t thumbnail_offset;
        uint16_t unknown_2;
        uint16_t unknown_3;
        int16_t width; // mm / 10
        int16_t height; // mm / 10
        uint16_t unknown_4;
        uint16_t unknown_5;
        uint8_t unknown_6[4];
    };
} pec_section2_t;

/** PEC RGB color. */
struct pec_rgb {
    int r; /** Red. */
    int g; /** Green. */
    int b; /** Blue. */
};

/** PEC thread. */
struct pec_thread {
    int index;          /** Thread index. */
    const char *id;     /** Thread id. */
    const char *code;   /** Thread code. */
    const char *name;   /** Name of thrad. */
    const char *type;   /** Type of thrad. */
    struct pec_rgb rgb; /** RGB color. */
};

static const struct pec_thread palette_thread_list[] = {
    {  0, "00", "000", "Undefined",         "A", { 220, 220, 220 } },
    {  1,  "1", "000", "Prussian Blue",     "A", {  26,  10, 148 } },
    {  2,  "2", "000", "Blue",              "A", {  15, 117, 255 } },
    {  3,  "3", "000", "Teal Green",        "A", {   0, 147,  76 } },
    {  4,  "4", "000", "Corn Flower Blue",  "A", { 186, 189, 254 } },
    {  5,  "5", "000", "Red",               "A", { 236,   0,   0 } },
    {  6,  "6", "000", "Reddish Brown",     "A", { 228, 153,  90 } },
    {  7,  "7", "000", "Magenta",           "A", { 204,  72, 171 } },
    {  8,  "8", "000", "Light Lilac",       "A", { 253, 196, 250 } },
    {  9,  "9", "000", "Lilac",             "A", { 221, 132, 205 } },
    { 10, "10", "000", "Mint Green",        "A", { 107, 211, 138 } },
    { 11, "11", "000", "Deep Gold",         "A", { 228, 169,  69 } },
    { 12, "12", "000", "Orange",            "A", { 255, 189,  66 } },
    { 13, "13", "000", "Yellow",            "A", { 255, 230,   0 } },
    { 14, "14", "000", "Lime Green",        "A", { 108, 217,   0 } },
    { 15, "15", "000", "Brass",             "A", { 193, 169,  65 } },
    { 16, "16", "000", "Silver",            "A", { 181, 173, 151 } },
    { 17, "17", "000", "Russet Brown",      "A", { 186, 156,  95 } },
    { 18, "18", "000", "Cream Brown",       "A", { 250, 245, 158 } },
    { 19, "19", "000", "Pewter",            "A", { 128, 128, 128 } },
    { 20, "20", "000", "Black",             "A", {   0,   0,   0 } },
    { 21, "21", "000", "Ultramarine",       "A", {   0,  28, 223 } },
    { 22, "22", "000", "Royal Purple",      "A", { 223,   0, 184 } },
    { 23, "23", "000", "Dark Gray",         "A", {  98,  98,  98 } },
    { 24, "24", "000", "Dark Brown",        "A", { 105,  38,  13 } },
    { 25, "25", "000", "Deep Rose",         "A", { 255,   0,  96 } },
    { 26, "26", "000", "Light Brown",       "A", { 191, 130,   0 } },
    { 27, "27", "000", "Salmon Pink",       "A", { 243, 145, 120 } },
    { 28, "28", "000", "Vermillion",        "A", { 255, 104,   5 } },
    { 29, "29", "000", "White",             "A", { 240, 240, 240 } },
    { 30, "30", "000", "Violet",            "A", { 200,  50, 205 } },
    { 31, "31", "000", "Seacrest",          "A", { 176, 191, 155 } },
    { 32, "32", "000", "Sky Blue",          "A", { 101, 191, 235 } },
    { 33, "33", "000", "Pumpkin",           "A", { 255, 186,   4 } },
    { 34, "34", "000", "Cream Yellow",      "A", { 255, 240, 108 } },
    { 35, "35", "000", "Khaki",             "A", { 254, 202,  21 } },
    { 36, "36", "000", "Clay Brown",        "A", { 243, 129,   1 } },
    { 37, "37", "000", "Leaf Green",        "A", {  55, 169,  35 } },
    { 38, "38", "000", "Peacock Blue",      "A", {  35,  70,  95 } },
    { 39, "39", "000", "Gray",              "A", { 166, 166, 149 } },
    { 40, "40", "000", "Warm Gray",         "A", { 206, 191, 166 } },
    { 41, "41", "000", "Dark Olive",        "A", { 150, 170,   2 } },
    { 42, "42", "000", "Linen",             "A", { 255, 227, 198 } },
    { 43, "43", "000", "Pink",              "A", { 255, 153, 215 } },
    { 44, "44", "000", "Deep Green",        "A", {   0, 112,   4 } },
    { 45, "45", "000", "Lavender",          "A", { 237, 204, 251 } },
    { 46, "46", "000", "Wisteria Violet",   "A", { 192, 137, 216 } },
    { 47, "47", "000", "Beige",             "A", { 231, 217, 180 } },
    { 48, "48", "000", "Carmine",           "A", { 233,  14, 134 } },
    { 49, "49", "000", "Amber Red",         "A", { 207, 104,  41 } },
    { 50, "50", "000", "Olive Green",       "A", {  64, 134,  21 } },
    { 51, "51", "000", "Dark Fuschia",      "A", { 219,  23, 151 } },
    { 52, "52", "000", "Tangerine",         "A", { 255, 167,   4 } },
    { 53, "53", "000", "Light Blue",        "A", { 185, 255, 255 } },
    { 54, "54", "000", "Emerald Green",     "A", {  34, 137,  39 } },
    { 55, "55", "000", "Purple",            "A", { 182,  18, 205 } },
    { 56, "56", "000", "Moss Green",        "A", {   0, 170,   0 } },
    { 57, "57", "000", "Flesh Pink",        "A", { 254, 169, 220 } },
    { 58, "58", "000", "Harvest Gold",      "A", { 254, 213,  16 } },
    { 59, "59", "000", "Electric Blue",     "A", {   0, 151, 223 } },
    { 60, "60", "000", "Lemon Yellow",      "A", { 255, 255, 132 } },
    { 61, "61", "000", "Fresh Green",       "A", { 207, 231, 116 } },
    { 62, "62", "000", "Applique Material", "A", { 255, 200, 100 } },
    { 63, "63", "000", "Applique Position", "A", { 255, 200, 200 } },
    { 64, "64", "000", "Applique",          "A", { 255, 200, 200 } }
};

static pec_section1_t pec_1;
static pec_section2_t pec_2;
static int32_t first_color = -1;
static int32_t color_idx;

static const char *get_thread_color (embroidery_thread_color_t color)
{
    return palette_thread_list[color >= sizeof(palette_thread_list) / sizeof(struct pec_thread) ? 0 : color].name;
}

static bool get_stitch (stitch_t *stitch, vfs_file_t *file)
{
    uint8_t cmd;
    bool eof;
    int16_t dx = 0, dy = 0;

    if(first_color != -1) {

        stitch->type = Stitch_Stop;
        stitch->color = (embroidery_thread_color_t)first_color;
        stitch->target.x = stitch->target.y = 0.0f;
        first_color = -1;

        return true;
    }

    vfs_read(&cmd, 1, 1, file);

    stitch->type = Stitch_Normal;

    if(!(eof = cmd == 0xFF)) {

        // parse x
        if(cmd & 0x80) {

            if(cmd == 0xFE) {

                stitch->type = Stitch_Stop;
                vfs_read(&cmd, 1, 1, file);
                vfs_read(&cmd, 1, 1, file);
                color_idx++;
                stitch->color = (embroidery_thread_color_t)pec_1.palette_index[color_idx];
                stitch->target.x = stitch->target.y = 0.0f;
                return true;

            } else {

                stitch->type = (cmd & 0x20) ? Stitch_Trim : ((cmd & 0x10) ? Stitch_Jump : Stitch_Normal);

//                if(stitch->type == Stitch_Trim)
//                    stitch->trims++;

                dx = (cmd & 0xF) << 8;
                vfs_read(&cmd, 1, 1, file);
                if((dx |= cmd) > 0x7ff)
                    dx -= 0x1000;
            }

            vfs_read(&cmd, 1, 1, file);

        } else {
            if((dx = cmd) > 0x3F)
                dx -= 0x80;
            vfs_read(&cmd, 1, 1, file);
        }

        //parse y
        if(cmd & 0x80) {

            stitch->type = (cmd & 0x20) ? Stitch_Trim : ((cmd & 0x10) ? Stitch_Jump : Stitch_Normal);

            dy = (cmd & 0xF) << 8;
            vfs_read(&cmd, 1, 1, file);
            if((dy |= cmd) > 0x7ff)
                dy -= 0x1000;

        } else {
            if((dy = cmd) > 0x3F)
                dy -= 0x80;
        }

        stitch->target.x = (float)dx / 10.0f;
        stitch->target.y = (float)-dy / 10.0f;
    }

    return !eof;
}

bool brother_open_file (vfs_file_t *file, embroidery_t *api)
{
    bool ok = false;
    pes_header_t header;

    if(vfs_read(header.buf, sizeof(pes_header_t), 1, file) == sizeof(pes_header_t) && !strncmp(header.version, "#PES", 4)) {

        vfs_seek(file, header.pec_offset);

        vfs_read(pec_1.buf, sizeof(pec_section1_t), 1, file);
        vfs_read(pec_2.buf, sizeof(pec_section2_t), 1, file);

        color_idx = 0;

        api->name = pec_1.label;
        api->size.x = pec_2.width;
        api->size.y = pec_2.height;

        api->get_stitch = get_stitch;
        api->get_thread_color = get_thread_color;
        first_color = (embroidery_thread_color_t)pec_1.palette_index[0];

        ok = true;
    } else
        vfs_seek(file, 0);

    return ok;
}

#endif // EMBROIDERY_ENABLE
