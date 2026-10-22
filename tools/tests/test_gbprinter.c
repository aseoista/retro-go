// Host test for the gnuboy Game Boy Printer emulation.
// Build & run: gcc -Wall -I retro-core/components/gnuboy tools/tests/test_gbprinter.c retro-core/components/gnuboy/printer.c -o /tmp/test_gbprinter && /tmp/test_gbprinter
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "printer.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)

static int pages = 0;
static int page_height = 0;
static uint8_t page_pixels[GB_PRINTER_WIDTH * 2048];

static void page_cb(const uint8_t *pixels, int width, int height)
{
    CHECK(width == GB_PRINTER_WIDTH);
    memcpy(page_pixels, pixels, width * height);
    page_height = height;
    pages++;
}

// Sends a packet, returns (alive << 8) | status
static int send_packet(byte cmd, byte comp, const byte *data, int len, int corrupt)
{
    uint16_t sum = cmd + comp + (len & 0xFF) + (len >> 8);
    byte header[] = {0x88, 0x33, cmd, comp, len & 0xFF, len >> 8};
    for (int i = 0; i < 6; i++)
        CHECK(gb_printer_transfer(header[i]) == 0x00);
    for (int i = 0; i < len; i++)
    {
        sum += data[i];
        CHECK(gb_printer_transfer(data[i]) == 0x00);
    }
    sum += corrupt;
    CHECK(gb_printer_transfer(sum & 0xFF) == 0x00);
    CHECK(gb_printer_transfer(sum >> 8) == 0x00);
    int alive = gb_printer_transfer(0x00);
    int status = gb_printer_transfer(0x00);
    return (alive << 8) | status;
}

static void ticks(int n)
{
    while (n--)
        gb_printer_tick();
}

// 2 tile rows where pixel x has color (x & 3)
static void make_stripes(byte *data)
{
    for (int i = 0; i < 640; i += 2)
    {
        data[i] = 0x55;
        data[i + 1] = 0x33;
    }
}

static int print_cmd(byte margins, byte palette)
{
    byte args[4] = {1, margins, palette, 0x40};
    return send_packet(0x02, 0, args, 4, 0);
}

int main(void)
{
    byte data[640];

    gb_printer_init(&page_cb);

    // Detection: STATUS answers alive + idle status
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8100);

    // Strip 1: uncompressed stripes, printed without feed (must not produce a page yet)
    CHECK(send_packet(0x01, 0, NULL, 0, 0) == 0x8100);
    make_stripes(data);
    CHECK(send_packet(0x04, 0, data, 640, 0) == 0x8100);
    CHECK(send_packet(0x04, 0, NULL, 0, 0) == 0x8100);
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8108); // unprocessed data
    CHECK(print_cmd(0x10, 0xE4) == 0x8108);
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8106); // busy
    ticks(30);
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8106);
    ticks(60);
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8104); // done
    CHECK(pages == 0);

    // Strip 2: RLE compressed solid black, inverted palette, feed after -> one merged page
    CHECK(send_packet(0x01, 0, NULL, 0, 0) == 0x8100); // INIT always answers 0, even after a print
    byte rle[] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80 | 120, 0xFF};
    CHECK(send_packet(0x04, 1, rle, sizeof(rle), 0) == 0x8100);
    CHECK(print_cmd(0x03, 0x1B) == 0x8100);
    CHECK(pages == 1);
    CHECK(page_height == 32);
    int ok = 1;
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 160; x++)
            ok &= page_pixels[y * 160 + x] == (x & 3);
    for (int y = 16; y < 32; y++)
        for (int x = 0; x < 160; x++)
            ok &= page_pixels[y * 160 + x] == 0; // color 3 through palette 0x1B is white
    CHECK(ok);
    ticks(60);

    // Bad checksum: flagged in the reply, packet ignored
    CHECK(send_packet(0x01, 0, NULL, 0, 0) == 0x8100);
    CHECK(send_packet(0x04, 0, data, 640, 1) == 0x8101);
    CHECK(send_packet(0x02, 0, (byte[]){1, 0x01, 0xE4, 0x40}, 4, 0) == 0x8100);
    CHECK(pages == 1); // nothing was pending, nothing to print
    ticks(60);

    // Idle flush: print without feed, then the link goes quiet
    CHECK(send_packet(0x01, 0, NULL, 0, 0) == 0x8100);
    CHECK(send_packet(0x04, 0, data, 640, 0) == 0x8100);
    CHECK(print_cmd(0x00, 0x00) == 0x8100); // palette 0 behaves as 0xE4
    ticks(119);
    CHECK(pages == 1);
    ticks(1);
    CHECK(pages == 2);
    CHECK(page_height == 16);
    CHECK(page_pixels[3] == 3);

    // Parser recovers from a truncated packet after a timeout
    gb_printer_transfer(0x88);
    gb_printer_transfer(0x33);
    gb_printer_transfer(0x04);
    ticks(30);
    CHECK(send_packet(0x0F, 0, NULL, 0, 0) == 0x8104);

    // Margin before flushes the previous page; reset flushes too
    CHECK(send_packet(0x04, 0, data, 640, 0) == 0x8104);
    CHECK(print_cmd(0x00, 0xE4) == 0x8104);
    CHECK(send_packet(0x04, 0, data, 640, 0) == 0x8106);
    CHECK(print_cmd(0x10, 0xE4) == 0x8106);
    CHECK(pages == 3 && page_height == 16);
    gb_printer_reset();
    CHECK(pages == 4 && page_height == 16);

    gb_printer_deinit();

    printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures != 0;
}
