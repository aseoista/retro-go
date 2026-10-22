#include <stdlib.h>
#include <string.h>
#include "printer.h"

/*
 * Game Boy Printer emulation.
 *
 * The Game Boy is always the master (internal clock) and the printer answers each byte. A packet is:
 *   0x88 0x33 <cmd> <compression> <len lo> <len hi> <data...> <checksum lo> <checksum hi> <0x00> <0x00>
 * The printer answers 0x00 to everything except the last two bytes, where it sends 0x81 (alive)
 * and its status byte. The checksum is the 16bit sum of everything from <cmd> to the end of <data>.
 *
 * Image data is sent as 2bpp tiles, 20 tiles per row (160 pixels). It accumulates in the printer
 * until a PRINT command applies a palette to it. Successive PRINT commands are merged into a single
 * page until one requests a feed after it (or before the next one), or the link goes idle.
 */

#define CMD_INIT    0x01
#define CMD_PRINT   0x02
#define CMD_DATA    0x04
#define CMD_BREAK   0x08
#define CMD_STATUS  0x0F

#define STATUS_CHECKSUM     0x01
#define STATUS_BUSY         0x02
#define STATUS_FULL         0x04
#define STATUS_UNPROCESSED  0x08
#define STATUS_PACKET_ERROR 0x10

#define PACKET_DATA_MAX     0x280 // 640 bytes = 2 rows of 20 tiles
#define TILE_ROW_BYTES      (20 * 16)
#define PENDING_MAX_LINES   (0x2000 / TILE_ROW_BYTES * 8) // The printer has 8KB of RAM
#define PAGE_MAX_LINES      2048
#define PAGE_GROW_LINES     144

#define BUSY_TICKS          60  // Time spent "printing" a strip, in frames
#define IDLE_TICKS          120 // Link inactivity after which a pending page is saved
#define PACKET_TIMEOUT      30  // Link inactivity after which a partial packet is discarded

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

enum
{
	ST_MAGIC1,
	ST_MAGIC2,
	ST_COMMAND,
	ST_COMPRESSION,
	ST_LENGTH_LO,
	ST_LENGTH_HI,
	ST_DATA,
	ST_CHECKSUM_LO,
	ST_CHECKSUM_HI,
	ST_ALIVE,
	ST_STATUS,
};

static struct
{
	gb_printer_page_cb_t *callback;

	// Packet parser
	int state;
	byte command;
	byte compression;
	uint16_t length;
	uint16_t received;
	uint16_t checksum;
	uint16_t sum;
	byte data[PACKET_DATA_MAX];
	byte reply;

	// Printer
	byte status;
	int busy;
	int idle;

	// Page: committed lines hold shades, the pending lines after them hold raw color indexes
	uint8_t *page;
	int page_lines;
	int pending_lines;
	int capacity;
} printer;


static bool reserve_lines(int lines)
{
	if (printer.capacity >= lines)
		return true;
	int capacity = MIN(lines + PAGE_GROW_LINES, PAGE_MAX_LINES);
	uint8_t *page = realloc(printer.page, capacity * GB_PRINTER_WIDTH);
	if (!page)
	{
		MESSAGE_ERROR("Out of memory (%d lines)\n", capacity);
		return false;
	}
	printer.page = page;
	printer.capacity = capacity;
	return true;
}


static void flush_page(void)
{
	if (printer.page_lines == 0)
		return;

	MESSAGE_INFO("Page complete: %dx%d\n", GB_PRINTER_WIDTH, printer.page_lines);

	if (printer.callback)
		printer.callback(printer.page, GB_PRINTER_WIDTH, printer.page_lines);

	// Keep the pending (not yet printed) lines
	memmove(printer.page, printer.page + printer.page_lines * GB_PRINTER_WIDTH,
		printer.pending_lines * GB_PRINTER_WIDTH);
	printer.page_lines = 0;
}


static int decompress(const byte *src, int src_len, byte *dst, int dst_max)
{
	int pos = 0, len = 0;
	while (pos < src_len && len < dst_max)
	{
		byte ctrl = src[pos++];
		if (ctrl & 0x80) // Run of a single byte
		{
			int count = (ctrl & 0x7F) + 2;
			if (pos >= src_len)
				break;
			while (count-- && len < dst_max)
				dst[len++] = src[pos];
			pos++;
		}
		else // Literal bytes
		{
			int count = ctrl + 1;
			while (count-- && pos < src_len && len < dst_max)
				dst[len++] = src[pos++];
		}
	}
	return len;
}


static void receive_data(void)
{
	byte buffer[PACKET_DATA_MAX];
	const byte *data = printer.data;
	int length = MIN(printer.length, PACKET_DATA_MAX);

	if (printer.compression)
	{
		length = decompress(printer.data, length, buffer, PACKET_DATA_MAX);
		data = buffer;
	}

	int tile_rows = length / TILE_ROW_BYTES;

	if (printer.pending_lines + tile_rows * 8 > PENDING_MAX_LINES)
	{
		MESSAGE_WARN("Printer buffer full!\n");
		printer.status |= STATUS_FULL;
		return;
	}

	if (printer.page_lines + printer.pending_lines + tile_rows * 8 > PAGE_MAX_LINES)
		flush_page();

	if (!reserve_lines(printer.page_lines + printer.pending_lines + tile_rows * 8))
		return;

	for (int row = 0; row < tile_rows; row++)
	{
		uint8_t *dst = printer.page + (printer.page_lines + printer.pending_lines) * GB_PRINTER_WIDTH;
		const byte *src = data + row * TILE_ROW_BYTES;

		for (int tile = 0; tile < 20; tile++)
		{
			for (int line = 0; line < 8; line++)
			{
				byte lo = src[tile * 16 + line * 2];
				byte hi = src[tile * 16 + line * 2 + 1];
				uint8_t *out = dst + line * GB_PRINTER_WIDTH + tile * 8;
				for (int bit = 7; bit >= 0; bit--)
					*out++ = ((lo >> bit) & 1) | (((hi >> bit) & 1) << 1);
			}
		}
		printer.pending_lines += 8;
	}
}


static void print(void)
{
	if (printer.length < 4)
	{
		printer.status |= STATUS_PACKET_ERROR;
		return;
	}

	int sheets = printer.data[0];
	int margin_before = printer.data[1] >> 4;
	int margin_after = printer.data[1] & 0xF;
	int palette = printer.data[2] ? printer.data[2] : 0xE4;

	MESSAGE_INFO("sheets=%d margins=%d/%d palette=%02X exposure=%02X lines=%d\n",
		sheets, margin_before, margin_after, palette, printer.data[3], printer.pending_lines);

	if (margin_before > 0)
		flush_page();

	if (sheets > 0)
	{
		uint8_t *pixel = printer.page + printer.page_lines * GB_PRINTER_WIDTH;
		uint8_t *end = pixel + printer.pending_lines * GB_PRINTER_WIDTH;
		while (pixel < end)
		{
			*pixel = (palette >> (*pixel * 2)) & 3;
			pixel++;
		}
		printer.page_lines += printer.pending_lines;
		printer.pending_lines = 0;
	}

	if (margin_after > 0)
		flush_page();

	printer.status = (printer.status & ~STATUS_UNPROCESSED) | STATUS_BUSY | STATUS_FULL;
	printer.busy = BUSY_TICKS;
}


static void process_packet(void)
{
	printer.reply = printer.status;

	if (printer.checksum != printer.sum)
	{
		MESSAGE_WARN("Checksum error (cmd=%02X)\n", printer.command);
		printer.reply |= STATUS_CHECKSUM;
		return;
	}

	printer.status &= ~(STATUS_CHECKSUM | STATUS_PACKET_ERROR);

	if (printer.length > PACKET_DATA_MAX)
	{
		printer.status |= STATUS_PACKET_ERROR;
		return;
	}

	switch (printer.command)
	{
	case CMD_INIT:
		printer.status = 0;
		printer.busy = 0;
		printer.pending_lines = 0;
		// Games (e.g. Pokemon) report a connection error if INIT doesn't answer 0, even after a print
		printer.reply = 0;
		break;

	case CMD_PRINT:
		print();
		break;

	case CMD_DATA:
		if (printer.length > 0)
			receive_data();
		else if (printer.pending_lines > 0) // An empty data packet marks the end of the image
			printer.status |= STATUS_UNPROCESSED;
		break;

	case CMD_BREAK:
		printer.status &= ~STATUS_BUSY;
		printer.busy = 0;
		break;

	case CMD_STATUS:
		break;

	default:
		MESSAGE_WARN("Unknown command %02X\n", printer.command);
		printer.status |= STATUS_PACKET_ERROR;
		break;
	}
}


byte gb_printer_transfer(byte data)
{
	byte out = 0x00;

	printer.idle = 0;

	switch (printer.state)
	{
	case ST_MAGIC1:
		if (data == 0x88)
			printer.state = ST_MAGIC2;
		break;

	case ST_MAGIC2:
		if (data == 0x33)
			printer.state = ST_COMMAND;
		else if (data != 0x88)
			printer.state = ST_MAGIC1;
		break;

	case ST_COMMAND:
		printer.command = data;
		printer.sum = data;
		printer.state = ST_COMPRESSION;
		break;

	case ST_COMPRESSION:
		printer.compression = data;
		printer.sum += data;
		printer.state = ST_LENGTH_LO;
		break;

	case ST_LENGTH_LO:
		printer.length = data;
		printer.sum += data;
		printer.state = ST_LENGTH_HI;
		break;

	case ST_LENGTH_HI:
		printer.length |= data << 8;
		printer.sum += data;
		printer.received = 0;
		printer.state = printer.length ? ST_DATA : ST_CHECKSUM_LO;
		break;

	case ST_DATA:
		if (printer.received < PACKET_DATA_MAX)
			printer.data[printer.received] = data;
		printer.sum += data;
		if (++printer.received == printer.length)
			printer.state = ST_CHECKSUM_LO;
		break;

	case ST_CHECKSUM_LO:
		printer.checksum = data;
		printer.state = ST_CHECKSUM_HI;
		break;

	case ST_CHECKSUM_HI:
		printer.checksum |= data << 8;
		process_packet();
		printer.state = ST_ALIVE;
		break;

	case ST_ALIVE:
		out = 0x81;
		printer.state = ST_STATUS;
		break;

	case ST_STATUS:
		out = printer.reply;
		printer.state = ST_MAGIC1;
		break;
	}

	return out;
}


void gb_printer_tick(void)
{
	if (printer.busy > 0 && --printer.busy == 0)
		printer.status &= ~STATUS_BUSY;

	if (printer.idle < IDLE_TICKS)
		printer.idle++;

	if (printer.idle == PACKET_TIMEOUT && printer.state != ST_MAGIC1)
	{
		MESSAGE_WARN("Packet timeout\n");
		printer.state = ST_MAGIC1;
	}

	if (printer.idle == IDLE_TICKS && printer.busy == 0)
		flush_page();
}


void gb_printer_flush(void)
{
	flush_page();
}


void gb_printer_reset(void)
{
	flush_page();
	printer.state = ST_MAGIC1;
	printer.status = 0;
	printer.reply = 0;
	printer.busy = 0;
	printer.idle = 0;
	printer.pending_lines = 0;
}


bool gb_printer_init(gb_printer_page_cb_t *page_callback)
{
	gb_printer_deinit();
	printer.callback = page_callback;
	gb_printer_reset();
	return true;
}


void gb_printer_deinit(void)
{
	free(printer.page);
	memset(&printer, 0, sizeof(printer));
}


const gb_link_device_t gb_printer_device = {
	.transfer = gb_printer_transfer,
	.tick = gb_printer_tick,
	.reset = gb_printer_reset,
};
