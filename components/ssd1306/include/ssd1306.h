#ifndef MAIN_SSD1306_H_
#define MAIN_SSD1306_H_

#include <cstdint>
#include <string>
#include <cstring>
#include <vector>
#include <driver/i2c.h>
#include <esp_log.h>

// Following definitions are borrowed from 
// http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

/* Control byte for i2c
Co : bit 8 : Continuation Bit 
 * 1 = no-continuation (only one byte to follow) 
 * 0 = the controller should expect a stream of bytes. 
D/C# : bit 7 : Data/Command Select bit 
 * 1 = the next byte or byte stream will be Data. 
 * 0 = a Command byte or byte stream will be coming up next. 
 Bits 6-0 will be all zeros. 
Usage: 
0x80 : Single Command byte 
0x00 : Command Stream 
0xC0 : Single Data byte 
0x40 : Data Stream
 */
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_SINGLE   0xC0
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20
#define OLED_CMD_SET_HORI_ADDR_MODE     0x00    // Horizontal Addressing Mode
#define OLED_CMD_SET_VERT_ADDR_MODE     0x01    // Vertical Addressing Mode
#define OLED_CMD_SET_PAGE_ADDR_MODE     0x02    // Page Addressing Mode
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP_0    0xA0    
#define OLED_CMD_SET_SEGMENT_REMAP_1    0xA1    
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

// Scrolling Command
#define OLED_CMD_HORIZONTAL_RIGHT       0x26
#define OLED_CMD_HORIZONTAL_LEFT        0x27
#define OLED_CMD_CONTINUOUS_SCROLL      0x29
#define OLED_CMD_DEACTIVE_SCROLL        0x2E
#define OLED_CMD_ACTIVE_SCROLL          0x2F
#define OLED_CMD_VERTICAL               0xA3

#define OLED_I2C_ADDRESS                0x3C

/** SSD1306 class
 *
 *  This is a driver code for the SSD1306 oled display controller
 *  This class provides interface for SSD1306 operation and accessing its registers.
 *
 *  Example:
 *  @code
 *
 *  @endcode
 */
class SSD1306 {
public:

    typedef enum {
        SCROLL_RIGHT = 1,
        SCROLL_LEFT = 2,
        SCROLL_DOWN = 3,
        SCROLL_UP = 4,
        SCROLL_STOP = 5,
    } scroll_type_t;

    typedef struct {
        bool _valid; // Not using it anymore
        int _segLen; // Not using it anymore
        uint8_t _segs[128];
    } PAGE_t;

    /** Create a PCA9629 instance connected to specified I2C pins with specified address
     *            
     * @param i2c_port The I2C port to use (default: 0)
     * @param i2c_address I2C-bus address (default: 0x20)     
     */
    SSD1306(const i2c_port_t i2c_port, const uint8_t i2c_address, const int width, const int height);
    ~SSD1306();

    void init(void);

    int get_width(void);
    int get_height(void);
    int get_pages(void);
    void show_buffer(void);
    void set_buffer(uint8_t * buffer);
    void get_buffer(uint8_t * buffer);
    void display_image(int page, int seg, uint8_t * images, int width);
    void display_text(int page, const std::string text, bool invert);
    void display_text_x3(int page, const std::string text, bool invert);
    void clear_screen(bool invert);
    void clear_line(int page, bool invert);
    void contrast(int contrast);
    void software_scroll(int start, int end);
    void scroll_text(const std::string text, bool invert);
    void scroll_clear();
    void hardware_scroll(scroll_type_t scroll);
    void wrap_arround(scroll_type_t scroll, int start, int end, int8_t delay);
    void bitmaps(int xpos, int ypos, uint8_t * bitmap, int width, int height, bool invert);    
    void invert(uint8_t *buf, size_t blen);
    void flip(uint8_t *buf, size_t blen);
    
    void fadeout();
    void dump();
    void dump_page(int page, int seg);

private:
    void pixel(int xpos, int ypos, bool invert);
    void line(int x1, int y1, int x2, int y2, bool invert);
    uint8_t copy_bit(uint8_t src, int srcBits, uint8_t dst, int dstBits);
    uint8_t rotate_byte(uint8_t ch1);
    
    void i2c_init(void);
    void i2c_display_image(int page, int seg, uint8_t * images, int width);
    void i2c_contrast(int contrast);
    void i2c_hardware_scroll(scroll_type_t scroll);    

    i2c_port_t _port;
    int _address;
    int _width;
    int _height;
    int _pages;
    int _dc;
    bool _scEnable;
    int _scStart;
    int _scEnd;
    int _scDirection;
    PAGE_t _page[8];
    bool _flip;
};

#endif /* MAIN_SSD1306_H_ */

