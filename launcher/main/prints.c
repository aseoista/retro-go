#include <rg_system.h>
#include <stdlib.h>
#include <string.h>

#include "gui.h"
#include "prints.h"

// Pictures printed by the emulated Game Boy Printer, stored as RG_BASE_PATH_PRINTS/<game>/NNNN.png

#define PRINTER_APP "printer"

static char **names = NULL;
static size_t names_count = 0;
static size_t names_capacity = 0;
static const char *current_path = NULL; // NULL when listing games

static int scan_folder_cb(const rg_scandir_t *entry, void *arg)
{
    if (current_path ? !(entry->is_file && rg_extension_match(entry->basename, "png")) : !entry->is_dir)
        return RG_SCANDIR_CONTINUE;

    if (names_count + 1 > names_capacity)
    {
        size_t new_capacity = (names_capacity * 1.5) + 1;
        char **new_buf = realloc(names, new_capacity * sizeof(char *));
        if (!new_buf)
            return RG_SCANDIR_STOP;
        names = new_buf;
        names_capacity = new_capacity;
    }

    names[names_count++] = strdup(entry->basename);

    return RG_SCANDIR_CONTINUE;
}

static void scan_folder(tab_t *tab, const char *path, const char *selection)
{
    char status[24];

    for (size_t i = 0; i < names_count; ++i)
        free(names[i]);
    names_count = 0;

    current_path = path ? rg_unique_string(path) : NULL;

    memset(&tab->status, 0, sizeof(tab->status));
    tab->navpath = current_path;

    rg_storage_scandir(current_path ? current_path : RG_BASE_PATH_PRINTS, scan_folder_cb, NULL, 0);

    gui_resize_list(tab, names_count);
    tab->listbox.cursor = 0;

    for (size_t i = 0; i < names_count; i++)
    {
        listbox_item_t *item = &tab->listbox.items[i];
        snprintf(item->text, sizeof(item->text), "%s", names[i]);
        if (current_path) // Hide the .png extension
            item->text[strlen(item->text) - 4] = 0;
        item->arg = names[i];
    }

    // Games in alphabetical order, prints newest first
    tab->listbox.sort_mode = current_path ? SORT_TEXT_DESC : SORT_TEXT_ASC;
    gui_sort_list(tab);

    if (selection)
    {
        for (int i = 0; i < tab->listbox.length; i++)
        {
            if (strcmp(tab->listbox.items[i].arg, selection) == 0)
            {
                tab->listbox.cursor = i;
                break;
            }
        }
    }

    if (current_path)
        snprintf(status, sizeof(status), _("%d prints"), (int)names_count);
    else
        snprintf(status, sizeof(status), _("%d games"), (int)names_count);
    gui_set_status(tab, status, "");

    gui_scroll_list(tab, SCROLL_SET, tab->listbox.cursor);
}

static const char *get_print_path(tab_t *tab, int index)
{
    static char path[RG_PATH_MAX + 1];
    if (!current_path || index < 0 || index >= tab->listbox.length)
        return NULL;
    snprintf(path, sizeof(path), "%s/%s", current_path, (char *)tab->listbox.items[index].arg);
    return path;
}

static void load_preview(tab_t *tab)
{
    const char *path = get_print_path(tab, tab->listbox.cursor);
    rg_image_t *img = path ? rg_surface_load_image_file(path, 0) : NULL;
    int max_height = gui.height * 0.70f; // Same as PREVIEW_HEIGHT in gui.c

    // gui_draw_preview doesn't preserve the aspect ratio, tall prints must be shrunk beforehand
    if (img && img->height > max_height)
    {
        rg_image_t *resized = rg_surface_resize(img, img->width * max_height / img->height, max_height);
        rg_surface_free(img);
        img = resized;
    }

    gui_set_preview(tab, img);
}

// Returns true if the print was deleted
static bool show_print_menu(const char *path)
{
    bool have_printer = rg_system_have_app(PRINTER_APP);
    const rg_gui_option_t options[] = {
        {1, _("Print via Bluetooth"), NULL, have_printer ? RG_DIALOG_FLAG_NORMAL : RG_DIALOG_FLAG_HIDDEN, NULL},
        {2, _("Delete file"), NULL, RG_DIALOG_FLAG_NORMAL, NULL},
        RG_DIALOG_END,
    };

    switch (rg_gui_dialog(rg_basename(path), options, 0))
    {
    case 1:
        gui_save_config();
        rg_system_switch_app(PRINTER_APP, PRINTER_APP, path, 0);
        break;

    case 2:
        if (rg_gui_confirm(_("Delete file"), rg_basename(path), false))
        {
            rg_storage_delete(path);
            return true;
        }
        break;
    }

    return false;
}

static void view_prints(tab_t *tab)
{
    rg_surface_t *screen = rg_surface_create(gui.width, gui.height, RG_PIXEL_565_LE, MEM_ANY);
    rg_image_t *image = NULL;
    uint32_t joystick = 0, prev_joystick = RG_KEY_ANY;
    int index = tab->listbox.cursor;
    int loaded = -1;
    int scroll = 0;
    bool redraw = true;

    if (!screen)
        return;

    while (index >= 0 && index < tab->listbox.length)
    {
        if (loaded != index)
        {
            rg_surface_free(image);
            image = rg_surface_load_image_file(get_print_path(tab, index), 0);
            loaded = index;
            scroll = 0;
            redraw = true;
        }

        int zoom = image ? RG_MAX(1, gui.width / image->width) : 1;
        int visible = gui.height / zoom;
        int max_scroll = image ? RG_MAX(0, image->height - visible) : 0;

        if (redraw)
        {
            scroll = RG_MIN(RG_MAX(scroll, 0), max_scroll);
            rg_surface_fill(screen, NULL, C_BLACK);
            if (image)
            {
                rg_rect_t src = {0, scroll, image->width, RG_MIN(visible, image->height - scroll)};
                rg_rect_t dst = {(gui.width - image->width * zoom) / 2, 0, image->width * zoom, src.height * zoom};
                rg_surface_copy(image, &src, screen, &dst, true);
            }
            rg_display_submit(screen, 0);
            redraw = false;
        }

        rg_task_delay(15);

        joystick = rg_input_read_gamepad();
        uint32_t pressed = joystick & ~prev_joystick;
        prev_joystick = joystick;

        // Up/Down scroll continuously while held, everything else acts once per press
        if (joystick & (RG_KEY_UP | RG_KEY_DOWN))
        {
            int step = (joystick & RG_KEY_UP) ? -4 : 4;
            if ((step < 0 && scroll > 0) || (step > 0 && scroll < max_scroll))
            {
                scroll += step;
                redraw = true;
            }
        }
        else if (pressed & RG_KEY_LEFT)
        {
            index = RG_MAX(index - 1, 0);
        }
        else if (pressed & RG_KEY_RIGHT)
        {
            index = RG_MIN(index + 1, tab->listbox.length - 1);
        }
        else if (pressed & (RG_KEY_A | RG_KEY_START | RG_KEY_SELECT))
        {
            const char *path = get_print_path(tab, index);
            if (path && show_print_menu(path))
            {
                char *selection = index > 0 ? strdup(tab->listbox.items[index - 1].arg) : NULL;
                scan_folder(tab, current_path, selection);
                free(selection);
                index = tab->listbox.cursor;
                loaded = -1;
                if (tab->listbox.length == 0) // That was the last print of this game
                {
                    rg_storage_delete(current_path);
                    scan_folder(tab, NULL, NULL);
                    break;
                }
            }
            prev_joystick = RG_KEY_ANY; // Ignore the key used to close the menu
            redraw = true;
        }
        else if (pressed & (RG_KEY_B | RG_KEY_MENU | RG_KEY_OPTION))
        {
            break;
        }
    }

    // Return to the list on the print we were looking at
    if (index >= 0 && index < tab->listbox.length)
        gui_scroll_list(tab, SCROLL_SET, index);

    rg_surface_free(image);
    rg_surface_free(screen);
    rg_input_wait_for_key(RG_KEY_ANY, false, 1000);
    gui_set_preview(tab, NULL);
}

static void event_handler(gui_event_t event, tab_t *tab)
{
    listbox_item_t *item = gui_get_selected_item(tab);
    const char *name = item ? item->arg : NULL;

    if (event == TAB_INIT)
    {
        scan_folder(tab, NULL, NULL);
    }
    else if (event == TAB_ENTER || event == TAB_LEAVE || event == TAB_SCROLL)
    {
        gui_set_preview(tab, NULL);
        tab->navpath = current_path;
    }
    else if (event == TAB_IDLE)
    {
        if (name && current_path && !tab->preview && gui.idle_counter == 1)
            load_preview(tab);
    }
    else if (event == TAB_ACTION)
    {
        if (name && current_path)
        {
            view_prints(tab);
        }
        else if (name)
        {
            char pathbuf[RG_PATH_MAX + 1];
            snprintf(pathbuf, sizeof(pathbuf), "%s/%s", RG_BASE_PATH_PRINTS, name);
            scan_folder(tab, pathbuf, NULL);
        }
    }
    else if (event == TAB_BACK)
    {
        if (current_path)
        {
            char *selection = strdup(rg_basename(current_path));
            gui_set_preview(tab, NULL);
            scan_folder(tab, NULL, selection);
            free(selection);
        }
    }
}

void prints_init(void)
{
    // The tab only exists once something has been printed
    if (!rg_storage_exists(RG_BASE_PATH_PRINTS))
        return;
    gui_add_tab("prints", _("Game Boy Printer"), NULL, event_handler);
}
