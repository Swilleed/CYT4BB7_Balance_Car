#ifndef __MENU_H
#define __MENU_H

typedef enum {
    submenu,    
    data,      
    function,
    dec    
} ItemType;

typedef struct {
    char name[20];
    ItemType type;
    float value;           
    int16_t subIndex;    
    void (*function)(void);  
} MenuItem;

typedef struct {
    char title[20];
    MenuItem item[8];  
    int16_t count;
    int16_t parIndex;
} Menu;

extern uint8_t Mission;
extern int8_t menuIndex,itemIndex,mode;

void Menu_Init(void);
void OLED_ShowMenu(void);
void up(void); 
void down(void);
void back(void);
void confirm(void);


void Key_Scanner(void);

#endif
