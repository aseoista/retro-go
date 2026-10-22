#include <rg_system.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"
#include "render.h"
#include "transport.h"

// Prints a picture saved by the emulated Game Boy Printer on a real (Bluetooth) thermal printer.
// The launcher starts us with the picture's path as argument and we return to it when done.

#define SETTING_ADDRESS  "PrinterAddress"
#define SETTING_NAME     "PrinterName"
#define SETTING_DARKNESS "Darkness"
#define SETTING_COPIES   "Copies"

#define FILE_ADDRESS     "file" // Pseudo printer: dump the data stream to the SD card
#define BAND_ROWS        64
#define MAX_DEVICES      16

static rg_app_t *app;
static rg_image_t *picture;
static char *printer_address;
static char *printer_name;
static int darkness;
static int copies;

static void draw_picture(void)
{
    rg_display_clear(C_BLACK);
    if (picture)
    {
        int max_height = rg_display_get_height() - 16;
        int height = RG_MIN(picture->height, max_height);
        int width = picture->width * height / picture->height;
        rg_gui_draw_image(8, (rg_display_get_height() - height) / 2, width, height, true, picture);
    }
}

static void event_handler(int event, void *arg)
{
    if (event == RG_EVENT_REDRAW)
        draw_picture();
}

static void set_printer(const char *address, const char *name)
{
    free(printer_address);
    free(printer_name);
    printer_address = strdup(address);
    printer_name = strdup(name);
    rg_settings_set_string(NS_APP, SETTING_ADDRESS, address);
    rg_settings_set_string(NS_APP, SETTING_NAME, name);
    rg_settings_commit();
}

static bool choose_printer(void)
{
    printer_device_t *devices = calloc(MAX_DEVICES, sizeof(printer_device_t));
    int count = 0;

    if (transport_spp.init())
    {
        rg_gui_draw_message(_("Searching for printers..."));
        count = RG_MAX(transport_spp.scan(devices, MAX_DEVICES, 10000), 0);
    }
    else
    {
        rg_gui_alert(_("Bluetooth"), _("Bluetooth initialization failed"));
    }

    rg_gui_option_t options[MAX_DEVICES + 3];
    rg_gui_option_t *option = options;

    // Devices that identify as printers first, but cheap printers often don't so we list everything
    for (int pass = 0; pass < 2; pass++)
    {
        for (int i = 0; i < count; i++)
        {
            if (devices[i].is_printer == (pass == 0))
                *option++ = (rg_gui_option_t){i + 1, devices[i].name, NULL, RG_DIALOG_FLAG_NORMAL, NULL};
        }
    }
    if (count == 0)
        *option++ = (rg_gui_option_t){0, _("No device found"), NULL, RG_DIALOG_FLAG_SKIP, NULL};
    *option++ = (rg_gui_option_t){-1, _("Save to SD card (debug)"), NULL, RG_DIALOG_FLAG_NORMAL, NULL};
    *option++ = (rg_gui_option_t)RG_DIALOG_END;

    intptr_t sel = rg_gui_dialog(_("Select printer"), options, 0);
    if (sel > 0)
        set_printer(devices[sel - 1].address, devices[sel - 1].name);
    else if (sel == -1)
        set_printer(FILE_ADDRESS, _("SD card (debug)"));

    free(devices);
    return sel != RG_DIALOG_CANCELLED;
}

static void print_picture(void)
{
    const printer_driver_t *driver = &driver_escpos;
    const printer_transport_t *transport = &transport_spp;
    char error[64] = {0};
    bool success = false;

    if (!printer_address[0] && !choose_printer())
        return;

    if (strcmp(printer_address, FILE_ADDRESS) == 0)
        transport = &transport_file;

    render_t *render = render_load(app->romPath, driver->head_width);
    uint8_t *band = malloc(BAND_ROWS * driver->head_width / 8);
    if (!render || !band)
    {
        snprintf(error, sizeof(error), "%s", _("Unable to load the picture"));
        goto done;
    }

    rg_gui_draw_message(_("Connecting to %s..."), printer_name);
    if (!transport->init() || !transport->open(printer_address))
    {
        snprintf(error, sizeof(error), "%s", _("Unable to connect to the printer"));
        goto done;
    }

    print_options_t options = {.darkness = darkness};
    int rows = render_rows_count(render);
    success = driver->begin_job(transport, &options);

    for (int copy = 0; copy < copies && success; copy++)
    {
        if (copy > 0)
            success = driver->feed(transport, 4);
        for (int row = 0; row < rows && success; row += BAND_ROWS)
        {
            int count = RG_MIN(BAND_ROWS, rows - row);
            rg_gui_draw_message(_("Printing... %d%%"), (copy * rows + row) * 100 / (copies * rows));
            render_rows(render, row, count, band);
            success = driver->send_rows(transport, band, count);
        }
    }

    if (success)
    {
        rg_gui_draw_message(_("Waiting for the printer..."));
        success = driver->end_job(transport, error, sizeof(error));
    }
    else
    {
        snprintf(error, sizeof(error), "%s", _("Connection lost"));
    }

    transport->close();

done:
    free(band);
    render_free(render);

    if (success)
    {
        rg_gui_draw_message(_("Done!"));
        rg_task_delay(1000);
        rg_system_exit();
    }

    rg_gui_alert(_("Printing failed"), error);
}

static rg_gui_event_t printer_cb(rg_gui_option_t *option, rg_gui_event_t event)
{
    if (event == RG_DIALOG_ENTER)
    {
        choose_printer();
        event = RG_DIALOG_REDRAW;
    }
    snprintf(option->value, 24, "%s", printer_address[0] ? printer_name : _("None"));
    return event == RG_DIALOG_REDRAW ? RG_DIALOG_REDRAW : RG_DIALOG_VOID;
}

static rg_gui_event_t darkness_cb(rg_gui_option_t *option, rg_gui_event_t event)
{
    if (event == RG_DIALOG_PREV && darkness > 1)
        darkness--;
    if (event == RG_DIALOG_NEXT && darkness < 15)
        darkness++;
    if (event == RG_DIALOG_PREV || event == RG_DIALOG_NEXT)
        rg_settings_set_number(NS_APP, SETTING_DARKNESS, darkness);
    sprintf(option->value, "%d", darkness);
    return RG_DIALOG_VOID;
}

static rg_gui_event_t copies_cb(rg_gui_option_t *option, rg_gui_event_t event)
{
    if (event == RG_DIALOG_PREV && copies > 1)
        copies--;
    if (event == RG_DIALOG_NEXT && copies < 5)
        copies++;
    if (event == RG_DIALOG_PREV || event == RG_DIALOG_NEXT)
        rg_settings_set_number(NS_APP, SETTING_COPIES, copies);
    sprintf(option->value, "%d", copies);
    return RG_DIALOG_VOID;
}

void app_main(void)
{
    const rg_handlers_t handlers = {
        .event = &event_handler,
    };

    app = rg_system_init(32000, &handlers, NULL);

    printer_address = rg_settings_get_string(NS_APP, SETTING_ADDRESS, "");
    printer_name = rg_settings_get_string(NS_APP, SETTING_NAME, "");
    darkness = RG_MIN(RG_MAX(rg_settings_get_number(NS_APP, SETTING_DARKNESS, 15), 1), 15);
    copies = RG_MIN(RG_MAX(rg_settings_get_number(NS_APP, SETTING_COPIES, 1), 1), 5);

    picture = rg_surface_load_image_file(app->romPath, 0);
    if (!picture)
    {
        rg_gui_alert(_("Printer"), _("Unable to load the picture"));
        rg_system_exit();
    }

    while (true)
    {
        const rg_gui_option_t options[] = {
            {1, _("Print"), NULL, RG_DIALOG_FLAG_NORMAL, NULL},
            {0, _("Printer"), "-", RG_DIALOG_FLAG_NORMAL, &printer_cb},
            {0, _("Darkness"), "-", RG_DIALOG_FLAG_NORMAL, &darkness_cb},
            {0, _("Copies"), "-", RG_DIALOG_FLAG_NORMAL, &copies_cb},
            RG_DIALOG_SEPARATOR,
            {2, _("Back"), NULL, RG_DIALOG_FLAG_NORMAL, NULL},
            RG_DIALOG_END,
        };

        draw_picture();
        intptr_t sel = rg_gui_dialog(rg_basename(app->romPath), options, 0);

        if (sel == 1)
            print_picture();
        else
            break;
    }

    rg_settings_commit();
    rg_system_exit();
}
