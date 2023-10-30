#include <string>
#include "i2c_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include "include/ssd1306.h"
#include "include/font8x8_basic.h"

#define TAG "ssd1306"

#define PACK8 __attribute__((aligned( __alignof__( uint8_t ) ), packed ))

typedef union out_column_t {
    uint32_t u32;
    uint8_t u8[4];
} PACK8 out_column_t;

SSD1306::SSD1306(const i2c_port_t port, const uint8_t address, const int width = 128, const int height = 64) {

    ESP_LOGD(TAG, "i2c_port: %d, i2c_address: %d", port, address);
    ESP_LOGD(TAG, "width: %d, height: %d", width, height);

    this->_port = port;
    this->_address = address;
    this->_width = width;
    this->_height = height;
}

SSD1306::~SSD1306() {

}

void SSD1306::init() {

    i2c_init();

    // Initialize internal buffer
    for (int i = 0; i < this->_pages; i++) {
        memset(this->_page[i]._segs, 0, 128);
    }
}

int SSD1306::get_width() {
    return this->_width;
}

int SSD1306::get_height() {
    return this->_height;
}

int SSD1306::get_pages() {
    return this->_pages;
}

void SSD1306::show_buffer() {

    for (int page = 0; page < this->_pages; page++) {
        i2c_display_image(page, 0, this->_page[page]._segs, this->_width);
    }

}

void SSD1306::set_buffer(uint8_t * buffer) {
    int index = 0;
    for (int page = 0; page < this->_pages; page++) {
        memcpy(&this->_page[page]._segs, &buffer[index], 128);
        index = index + 128;
    }
}

void SSD1306::get_buffer(uint8_t * buffer) {
    int index = 0;
    for (int page = 0; page < this->_pages; page++) {
        memcpy(&buffer[index], &this->_page[page]._segs, 128);
        index = index + 128;
    }
}

void SSD1306::display_image(int page, int seg, uint8_t * images, int width) {

    i2c_display_image(page, seg, images, width);

    // Set to internal buffer
    memcpy(&this->_page[page]._segs[seg], images, width);
}

void SSD1306::display_text(int page, const std::string text, bool invert) {
    if (page >= this->_pages) return;

    std::string newText;
    if (text.length() > 16) {
        newText = text.substr(0, 16);
    } else {
        newText = text;
    }

    uint8_t seg = 0;
    uint8_t image[8];
    for (uint8_t i = 0; i < text.length(); i++) {
        memcpy(image, font8x8_basic_tr[(uint8_t) newText.data()[i]], 8);
        if (invert) SSD1306::invert(image, 8);
        if (this->_flip) SSD1306::flip(image, 8);
        SSD1306::display_image(page, seg, image, 8);
        seg = seg + 8;
    }
}

// by Coert Vonk

void SSD1306::display_text_x3(int page, const std::string text, bool invert) {
    if (page >= this->_pages) return;

    std::string newText;
    if (text.length() > 5) {
        newText = text.substr(0, 5);
    } else {
        newText = text;
    }

    uint8_t seg = 0;

    for (uint8_t nn = 0; nn < text.length(); nn++) {

        uint8_t const * const in_columns = font8x8_basic_tr[(uint8_t) newText.data()[nn]];

        // make the character 3x as high
        out_column_t out_columns[8];
        memset(out_columns, 0, sizeof (out_columns));

        for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)

            uint32_t in_bitmask = 0b1;
            uint32_t out_bitmask = 0b111;

            for (uint8_t yy = 0; yy < 8; yy++) { // for pixel (y-direction)
                if (in_columns[xx] & in_bitmask) {
                    out_columns[xx].u32 |= out_bitmask;
                }
                in_bitmask <<= 1;
                out_bitmask <<= 3;
            }
        }

        // render character in 8 column high pieces, making them 3x as wide
        for (uint8_t yy = 0; yy < 3; yy++) { // for each group of 8 pixels high (y-direction)

            uint8_t image[24];
            for (uint8_t xx = 0; xx < 8; xx++) { // for each column (x-direction)
                image[xx * 3 + 0] =
                        image[xx * 3 + 1] =
                        image[xx * 3 + 2] = out_columns[xx].u8[yy];
            }
            if (invert) SSD1306::invert(image, 24);
            if (this->_flip) SSD1306::flip(image, 24);
            //if (this->_address == I2CAddress) {

            i2c_display_image(page + yy, seg, image, 24);

            memcpy(&this->_page[page + yy]._segs[seg], image, 24);
            //}
            seg = seg + 24;
        }
    }
}

void SSD1306::clear_screen(bool invert) {

    for (int page = 0; page < this->_pages; page++) {
        SSD1306::display_text(page, "                ", invert);
    }
}

void SSD1306::clear_line(int page, bool invert) {
    SSD1306::display_text(page, "                ", invert);
}

void SSD1306::contrast(int contrast) {

    i2c_contrast(contrast);

}

void SSD1306::software_scroll(int start, int end) {
    ESP_LOGD(TAG, "software_scroll start=%d end=%d _pages=%d", start, end, this->_pages);
    if (start < 0 || end < 0) {
        this->_scEnable = false;
    } else if (start >= this->_pages || end >= this->_pages) {
        this->_scEnable = false;
    } else {
        this->_scEnable = true;
        this->_scStart = start;
        this->_scEnd = end;
        this->_scDirection = 1;
        if (start > end) this->_scDirection = -1;
    }
}

void SSD1306::scroll_text(std::string text, bool invert) {
    ESP_LOGD(TAG, "this->_scEnable=%d", this->_scEnable);

    if (this->_scEnable == false) {
        return;
    }

    int srcIndex = this->_scEnd - this->_scDirection;

    while (1) {
        int dstIndex = srcIndex + this->_scDirection;
        ESP_LOGD(TAG, "srcIndex=%d dstIndex=%d", srcIndex, dstIndex);
        for (int seg = 0; seg < this->_width; seg++) {
            this->_page[dstIndex]._segs[seg] = this->_page[srcIndex]._segs[seg];
        }

        i2c_display_image(dstIndex, 0, this->_page[dstIndex]._segs, sizeof (this->_page[dstIndex]._segs));

        if (srcIndex == this->_scStart) {
            break;
        }

        srcIndex = srcIndex - this->_scDirection;
    }

    std::string newText;
    if (text.length() > 16) {
        newText = text.substr(0, 16);
    } else {
        newText = text;
    }

    SSD1306::display_text(srcIndex, newText, invert);
}

void SSD1306::scroll_clear() {
    ESP_LOGD(TAG, "this->_scEnable=%d", this->_scEnable);
    if (this->_scEnable == false) return;

    int srcIndex = this->_scEnd - this->_scDirection;
    while (1) {
        int dstIndex = srcIndex + this->_scDirection;
        ESP_LOGD(TAG, "srcIndex=%d dstIndex=%d", srcIndex, dstIndex);
        SSD1306::clear_line(dstIndex, false);
        if (dstIndex == this->_scStart) break;
        srcIndex = srcIndex - this->_scDirection;
    }
}

void SSD1306::hardware_scroll(SSD1306::scroll_type_t scroll) {

    i2c_hardware_scroll(scroll);

}

// delay = 0 : display with no wait
// delay > 0 : display with wait
// delay < 0 : no display

void SSD1306::wrap_arround(SSD1306::scroll_type_t scroll, int start, int end, int8_t delay) {
    if (scroll == SCROLL_RIGHT) {
        int _start = start; // 0 to 7
        int _end = end; // 0 to 7
        if (_end >= this->_pages) _end = this->_pages - 1;
        uint8_t wk;
        //for (int page=0;page<this->_pages;page++) {
        for (int page = _start; page <= _end; page++) {
            wk = this->_page[page]._segs[127];
            for (int seg = 127; seg > 0; seg--) {
                this->_page[page]._segs[seg] = this->_page[page]._segs[seg - 1];
            }
            this->_page[page]._segs[0] = wk;
        }

    } else if (scroll == SCROLL_LEFT) {
        int _start = start; // 0 to 7
        int _end = end; // 0 to 7
        if (_end >= this->_pages) _end = this->_pages - 1;
        uint8_t wk;
        //for (int page=0;page<this->_pages;page++) {
        for (int page = _start; page <= _end; page++) {
            wk = this->_page[page]._segs[0];
            for (int seg = 0; seg < 127; seg++) {
                this->_page[page]._segs[seg] = this->_page[page]._segs[seg + 1];
            }
            this->_page[page]._segs[127] = wk;
        }

    } else if (scroll == SCROLL_UP) {
        int _start = start; // 0 to {width-1}
        int _end = end; // 0 to {width-1}
        if (_end >= this->_width) _end = this->_width - 1;
        uint8_t wk0;
        uint8_t wk1;
        uint8_t wk2;
        uint8_t save[128];
        // Save pages 0
        for (int seg = 0; seg < 128; seg++) {
            save[seg] = this->_page[0]._segs[seg];
        }
        // Page0 to Page6
        for (int page = 0; page < this->_pages - 1; page++) {
            //for (int seg=0;seg<128;seg++) {
            for (int seg = _start; seg <= _end; seg++) {
                wk0 = this->_page[page]._segs[seg];
                wk1 = this->_page[page + 1]._segs[seg];
                if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);
                if (this->_flip) wk1 = SSD1306::rotate_byte(wk1);
                if (seg == 0) {
                    ESP_LOGD(TAG, "b page=%d wk0=%02x wk1=%02x", page, wk0, wk1);
                }
                wk0 = wk0 >> 1;
                wk1 = wk1 & 0x01;
                wk1 = wk1 << 7;
                wk2 = wk0 | wk1;
                if (seg == 0) {
                    ESP_LOGD(TAG, "a page=%d wk0=%02x wk1=%02x wk2=%02x", page, wk0, wk1, wk2);
                }
                if (this->_flip) wk2 = SSD1306::rotate_byte(wk2);
                this->_page[page]._segs[seg] = wk2;
            }
        }
        // Page7
        int pages = this->_pages - 1;
        //for (int seg=0;seg<128;seg++) {
        for (int seg = _start; seg <= _end; seg++) {
            wk0 = this->_page[pages]._segs[seg];
            wk1 = save[seg];
            if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);
            if (this->_flip) wk1 = SSD1306::rotate_byte(wk1);
            wk0 = wk0 >> 1;
            wk1 = wk1 & 0x01;
            wk1 = wk1 << 7;
            wk2 = wk0 | wk1;
            if (this->_flip) wk2 = SSD1306::rotate_byte(wk2);
            this->_page[pages]._segs[seg] = wk2;
        }

    } else if (scroll == SCROLL_DOWN) {
        int _start = start; // 0 to {width-1}
        int _end = end; // 0 to {width-1}
        if (_end >= this->_width) _end = this->_width - 1;
        uint8_t wk0;
        uint8_t wk1;
        uint8_t wk2;
        uint8_t save[128];
        // Save pages 7
        int pages = this->_pages - 1;
        for (int seg = 0; seg < 128; seg++) {
            save[seg] = this->_page[pages]._segs[seg];
        }
        // Page7 to Page1
        for (int page = pages; page > 0; page--) {
            //for (int seg=0;seg<128;seg++) {
            for (int seg = _start; seg <= _end; seg++) {
                wk0 = this->_page[page]._segs[seg];
                wk1 = this->_page[page - 1]._segs[seg];
                if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);
                if (this->_flip) wk1 = SSD1306::rotate_byte(wk1);
                if (seg == 0) {
                    ESP_LOGD(TAG, "b page=%d wk0=%02x wk1=%02x", page, wk0, wk1);
                }
                wk0 = wk0 << 1;
                wk1 = wk1 & 0x80;
                wk1 = wk1 >> 7;
                wk2 = wk0 | wk1;
                if (seg == 0) {
                    ESP_LOGD(TAG, "a page=%d wk0=%02x wk1=%02x wk2=%02x", page, wk0, wk1, wk2);
                }
                if (this->_flip) wk2 = SSD1306::rotate_byte(wk2);
                this->_page[page]._segs[seg] = wk2;
            }
        }
        // Page0
        //for (int seg=0;seg<128;seg++) {
        for (int seg = _start; seg <= _end; seg++) {
            wk0 = this->_page[0]._segs[seg];
            wk1 = save[seg];
            if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);
            if (this->_flip) wk1 = SSD1306::rotate_byte(wk1);
            wk0 = wk0 << 1;
            wk1 = wk1 & 0x80;
            wk1 = wk1 >> 7;
            wk2 = wk0 | wk1;
            if (this->_flip) wk2 = SSD1306::rotate_byte(wk2);
            this->_page[0]._segs[seg] = wk2;
        }

    }

    if (delay >= 0) {
        for (int page = 0; page < this->_pages; page++) {
            i2c_display_image(page, 0, this->_page[page]._segs, 128);
            if (delay) vTaskDelay(delay);
        }
    }

}

void SSD1306::bitmaps(int xpos, int ypos, uint8_t * bitmap, int width, int height, bool invert) {
    if ((width % 8) != 0) {
        ESP_LOGE(TAG, "width must be a multiple of 8");
        return;
    }
    int _width = width / 8;
    uint8_t wk0;
    uint8_t wk1;
    uint8_t wk2;
    uint8_t page = (ypos / 8);
    uint8_t _seg = xpos;
    uint8_t dstBits = (ypos % 8);
    ESP_LOGD(TAG, "ypos=%d page=%d dstBits=%d", ypos, page, dstBits);
    int offset = 0;
    for (int _height = 0; _height < height; _height++) {
        for (int index = 0; index < _width; index++) {
            for (int srcBits = 7; srcBits >= 0; srcBits--) {
                wk0 = this->_page[page]._segs[_seg];
                if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);

                wk1 = bitmap[index + offset];
                if (invert) wk1 = ~wk1;

                //wk2 = SSD1306::copy_bit(bitmap[index+offset], srcBits, wk0, dstBits);
                wk2 = SSD1306::copy_bit(wk1, srcBits, wk0, dstBits);
                if (this->_flip) wk2 = SSD1306::rotate_byte(wk2);

                ESP_LOGD(TAG, "index=%d offset=%d page=%d _seg=%d, wk2=%02x", index, offset, page, _seg, wk2);
                this->_page[page]._segs[_seg] = wk2;
                _seg++;
            }
        }
        vTaskDelay(1);
        offset = offset + _width;
        dstBits++;
        _seg = xpos;
        if (dstBits == 8) {
            page++;
            dstBits = 0;
        }
    }

#if 0
    for (int _seg = ypos; _seg < ypos + width; _seg++) {
        SSD1306::dump_page(page - 1, _seg);
    }
    for (int _seg = ypos; _seg < ypos + width; _seg++) {
        SSD1306::dump_page(page, _seg);
    }
#endif
    SSD1306::show_buffer();
}


// Set pixel to internal buffer. Not show it.

void SSD1306::pixel(int xpos, int ypos, bool invert) {
    uint8_t _page = (ypos / 8);
    uint8_t _bits = (ypos % 8);
    uint8_t _seg = xpos;
    uint8_t wk0 = this->_page[_page]._segs[_seg];
    uint8_t wk1 = 1 << _bits;
    ESP_LOGD(TAG, "ypos=%d _page=%d _bits=%d wk0=0x%02x wk1=0x%02x", ypos, _page, _bits, wk0, wk1);
    if (invert) {
        wk0 = wk0 & ~wk1;
    } else {
        wk0 = wk0 | wk1;
    }
    if (this->_flip) wk0 = SSD1306::rotate_byte(wk0);
    ESP_LOGD(TAG, "wk0=0x%02x wk1=0x%02x", wk0, wk1);
    this->_page[_page]._segs[_seg] = wk0;
}

// Set line to internal buffer. Not show it.

void SSD1306::line(int x1, int y1, int x2, int y2, bool invert) {
    int i;
    int dx, dy;
    int sx, sy;
    int E;

    /* distance between two points */
    dx = (x2 > x1) ? x2 - x1 : x1 - x2;
    dy = (y2 > y1) ? y2 - y1 : y1 - y2;

    /* direction of two point */
    sx = (x2 > x1) ? 1 : -1;
    sy = (y2 > y1) ? 1 : -1;

    /* inclination < 1 */
    if (dx > dy) {
        E = -dx;
        for (i = 0; i <= dx; i++) {
            SSD1306::pixel(x1, y1, invert);
            x1 += sx;
            E += 2 * dy;
            if (E >= 0) {
                y1 += sy;
                E -= 2 * dx;
            }
        }

        /* inclination >= 1 */
    } else {
        E = -dy;
        for (i = 0; i <= dy; i++) {
            SSD1306::pixel(x1, y1, invert);
            y1 += sy;
            E += 2 * dx;
            if (E >= 0) {
                x1 += sx;
                E -= 2 * dy;
            }
        }
    }
}

void SSD1306::invert(uint8_t *buf, size_t blen) {
    uint8_t wk;
    for (int i = 0; i < blen; i++) {
        wk = buf[i];
        buf[i] = ~wk;
    }
}

// Flip upside down

void SSD1306::flip(uint8_t *buf, size_t blen) {
    for (int i = 0; i < blen; i++) {
        buf[i] = SSD1306::rotate_byte(buf[i]);
    }
}

uint8_t SSD1306::copy_bit(uint8_t src, int srcBits, uint8_t dst, int dstBits) {
    ESP_LOGD(TAG, "src=%02x srcBits=%d dst=%02x dstBits=%d", src, srcBits, dst, dstBits);
    uint8_t smask = 0x01 << srcBits;
    uint8_t dmask = 0x01 << dstBits;
    uint8_t _src = src & smask;
#if 0
    if (_src != 0) _src = 1;
    uint8_t _wk = _src << dstBits;
    uint8_t _dst = dst | _wk;
#endif
    uint8_t _dst;
    if (_src != 0) {
        _dst = dst | dmask; // set bit
    } else {
        _dst = dst & ~(dmask); // clear bit
    }
    return _dst;
}


// Rotate 8-bit data
// 0x12-->0x48

uint8_t SSD1306::rotate_byte(uint8_t ch1) {
    uint8_t ch2 = 0;
    for (int j = 0; j < 8; j++) {
        ch2 = (ch2 << 1) + (ch1 & 0x01);
        ch1 = ch1 >> 1;
    }
    return ch2;
}

void SSD1306::fadeout() {

    uint8_t image[1];
    for (int page = 0; page < this->_pages; page++) {
        image[0] = 0xFF;
        for (int line = 0; line < 8; line++) {
            if (this->_flip) {
                image[0] = image[0] >> 1;
            } else {
                image[0] = image[0] << 1;
            }
            for (int seg = 0; seg < 128; seg++) {
                i2c_display_image(page, seg, image, 1);
                this->_page[page]._segs[seg] = image[0];
            }
        }
    }
}

void SSD1306::dump() {
    printf("_address=%d\n", this->_address);
    printf("_width=%d\n", this->_width);
    printf("_height=%d\n", this->_height);
    printf("_pages=%d\n", this->_pages);
}

void SSD1306::dump_page(int page, int seg) {
    ESP_LOGD(TAG, "this->_page[%d]._segs[%d]=%02x", page, seg, this->_page[page]._segs[seg]);
}

void SSD1306::i2c_init() {

    this->_flip = false;
    this->_pages = 8;
    if (this->_height == 32) this->_pages = 4;

    uint8_t data[27];
    data[0] = OLED_CONTROL_BYTE_CMD_STREAM;
    data[1] = OLED_CMD_DISPLAY_OFF; // AE
    data[2] = OLED_CMD_SET_MUX_RATIO; // AB
    if (this->_height == 64) {
        data[3] = 0x3f; // 0x3f = height 64 // 0x1f = height 32,
    } else {
        data[3] = 0x1f;
    }
    data[4] = OLED_CMD_SET_DISPLAY_OFFSET;
    data[5] = 0x00;
    data[6] = OLED_CMD_SET_DISPLAY_START_LINE;
    if (this->_flip) {
        data[7] = OLED_CMD_SET_SEGMENT_REMAP_0;
    } else {
        data[7] = OLED_CMD_SET_SEGMENT_REMAP_1;
    }
    data[8] = OLED_CMD_SET_COM_SCAN_MODE;
    data[9] = OLED_CMD_SET_DISPLAY_CLK_DIV;
    data[10] = 0x80;
    data[11] = OLED_CMD_SET_COM_PIN_MAP;
    if (this->_height == 64) {
        data[12] = 0x12; // height 64
    } else {
        data[12] = 0x02;
    }
    data[13] = OLED_CMD_SET_CONTRAST;
    data[14] = 0xff;
    data[15] = OLED_CMD_DISPLAY_RAM;
    data[16] = OLED_CMD_SET_VCOMH_DESELCT;
    data[17] = 0x40;
    data[18] = OLED_CMD_SET_MEMORY_ADDR_MODE;
    data[19] = OLED_CMD_SET_PAGE_ADDR_MODE;
    data[20] = 0x00;
    data[21] = 0x10;
    data[22] = OLED_CMD_SET_CHARGE_PUMP;
    data[23] = 0x14;
    data[24] = OLED_CMD_DEACTIVE_SCROLL;
    data[25] = OLED_CMD_DISPLAY_NORMAL;
    data[26] = OLED_CMD_DISPLAY_ON;

    // initialise display
    i2c_manager_write(this->_port, this->_address, I2C_NO_REG, data, 27);

}

void SSD1306::i2c_display_image(int page, int seg, uint8_t * images, int width) {
   
    if (page >= this->_pages) return;
    if (seg >= this->_width) return;

    int _seg = seg + CONFIG_OFFSETX;
    uint8_t columLow = _seg & 0x0F;
    uint8_t columHigh = (_seg >> 4) & 0x0F;

    int _page = page;
    if (this->_flip) {
        _page = (this->_pages - page) - 1;
    }

    uint8_t data[4];
    data[0] = OLED_CONTROL_BYTE_CMD_STREAM;
    data[1] = (0x00 + columLow);
    data[2] = (0x10 + columHigh);
    data[3] = (0xB0 | _page);

    i2c_manager_write(this->_port, this->_address, I2C_NO_REG, data, 4);

    uint8_t imagedata[width + 1];
    imagedata[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    memcpy(&imagedata[1], images, width);

    // FIXME: This line causes the ESP32 to core dump (but only if clear screen is being called)
    ESP_LOGI(TAG, "i2c_display_image: width (+1): %d", width + 1);
    i2c_manager_write(this->_port, this->_address, I2C_NO_REG, imagedata, width + 1);
}

void SSD1306::i2c_contrast(int contrast) {

    int _contrast = contrast;

    if (contrast < 0x0) _contrast = 0;
    if (contrast > 0xFF) _contrast = 0xFF;

    uint8_t data[3] = {
        OLED_CONTROL_BYTE_CMD_STREAM,
        OLED_CMD_SET_CONTRAST,
        (uint8_t) _contrast,
    };

    i2c_manager_write(this->_port, this->_address, I2C_NO_REG, data, 3);
}

//void SSD1306::i2c_hardware_scroll(scroll_type_t scroll) {
//    esp_err_t espRc;
//
//    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
//
//    if (scroll == SCROLL_RIGHT) {
//        i2c_master_write_byte(cmd, OLED_CMD_HORIZONTAL_RIGHT, true); // 26
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        i2c_master_write_byte(cmd, 0x07, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); //
//        i2c_master_write_byte(cmd, 0xFF, true); //
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_LEFT) {
//        i2c_master_write_byte(cmd, OLED_CMD_HORIZONTAL_LEFT, true); // 27
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        i2c_master_write_byte(cmd, 0x07, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); //
//        i2c_master_write_byte(cmd, 0xFF, true); //
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_DOWN) {
//        i2c_master_write_byte(cmd, OLED_CMD_CONTINUOUS_SCROLL, true); // 29
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        //i2c_master_write_byte(cmd, 0x01, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x3F, true); // Vertical scrolling offset
//
//        i2c_master_write_byte(cmd, OLED_CMD_VERTICAL, true); // A3
//        i2c_master_write_byte(cmd, 0x00, true);
//        if (this->_height == 64)
//            //i2c_master_write_byte(cmd, 0x7F, true);
//            i2c_master_write_byte(cmd, 0x40, true);
//        if (this->_height == 32)
//            i2c_master_write_byte(cmd, 0x20, true);
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_UP) {
//        i2c_master_write_byte(cmd, OLED_CMD_CONTINUOUS_SCROLL, true); // 29
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        //i2c_master_write_byte(cmd, 0x01, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x01, true); // Vertical scrolling offset
//
//        i2c_master_write_byte(cmd, OLED_CMD_VERTICAL, true); // A3
//        i2c_master_write_byte(cmd, 0x00, true);
//        if (this->_height == 64)
//            //i2c_master_write_byte(cmd, 0x7F, true);
//            i2c_master_write_byte(cmd, 0x40, true);
//        if (this->_height == 32)
//            i2c_master_write_byte(cmd, 0x20, true);
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_STOP) {
//        i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true); // 2E
//    }
//
//    i2c_master_stop(cmd);
//    espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
//    if (espRc == ESP_OK) {
//        ESP_LOGD(tag, "Scroll command succeeded");
//    } else {
//        ESP_LOGE(tag, "Scroll command failed. code: 0x%.2X", espRc);
//    }
//
//    i2c_cmd_link_delete(cmd);
//}
