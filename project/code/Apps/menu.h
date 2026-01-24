#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>
#define MENU_MAX_CHILDREN 4

#define KEY_UP KEY_1
#define KEY_DOWN KEY_2
#define KEY_SELECT KEY_3
#define KEY_BACK KEY_4

typedef struct menu
{
    char *title;
    struct menu *parent;
    struct menu *children[MENU_MAX_CHILDREN];
    uint8_t child_count;
    void (*action)(void);
} Menu;

Menu *InitMenu(void);
void Menu_AddChild(Menu *parent, Menu *child);
void DisplayMenu(void);

#endif // __MENU_H