#include "zf_common_typedef.h"
#include <string.h>
#include "Menu.h"
#include "zf_device_oled.h"
#include "zf_device_key.h"
Menu menu[6];
uint8_t Mission;
uint8_t Save_State = 0;
void A(void){
  if(!Mission){Mission = 1;}
  else {Mission = 0;}
}
void B(void){
  if(!Mission){Mission = 2;}
  else {Mission = 0;}
}
void C(void){
  if(!Mission){Mission = 3;}
  else {Mission = 0;}
}
void D(void){
  if(!Mission){Mission = 4;}
  else {Mission = 0;}
}
void E(void){
  if(!Mission){Mission = 5;}
  else {Mission = 0;}
}
void Flash(void){
  
}

void Save(void){
   Save_State = !Save_State;
}

void Menu_Init(void)
{
    oled_init();
  
    strcpy(menu[0].title, "Main Menu");
    menu[0].count = 6;
    menu[0].parIndex = -1;
    strcpy(menu[0].item[0].name, "PID");
    menu[0].item[0].type = submenu;
    menu[0].item[0].value = 0;
    menu[0].item[0].subIndex = 1;
    menu[0].item[0].function = NULL;
    strcpy(menu[0].item[1].name, "A");
    menu[0].item[1].type = function;
    menu[0].item[1].value = 0;
    menu[0].item[1].subIndex = -1;
    menu[0].item[1].function = A;
    strcpy(menu[0].item[2].name, "B");
    menu[0].item[2].type = function;
    menu[0].item[2].value = 0;
    menu[0].item[2].subIndex = -1;
    menu[0].item[2].function = B;
    strcpy(menu[0].item[3].name, "C");
    menu[0].item[3].type = function;
    menu[0].item[3].value = 0;
    menu[0].item[3].subIndex = -1;
    menu[0].item[3].function = C;
    strcpy(menu[0].item[4].name, "D");
    menu[0].item[4].type = submenu;
    menu[0].item[4].value = 0;
    menu[0].item[4].subIndex = 5;
    menu[0].item[4].function = NULL;
    strcpy(menu[0].item[5].name, "E");
    menu[0].item[5].type = function;
    menu[0].item[5].value = 0;
    menu[0].item[5].subIndex = -1;
    menu[0].item[5].function = E;
    
    strcpy(menu[1].title, "PID");
    menu[1].count = 3;
    menu[1].parIndex = 0;
    strcpy(menu[1].item[0].name, "STAND");
    menu[1].item[0].type = submenu;
    menu[1].item[0].value = 0;
    menu[1].item[0].subIndex = 2;
    menu[1].item[0].function = NULL;
    strcpy(menu[1].item[1].name, "DIR");
    menu[1].item[1].type = submenu;
    menu[1].item[1].value = 0;
    menu[1].item[1].subIndex = 3;
    menu[1].item[1].function = NULL;
    strcpy(menu[1].item[2].name, "POS");
    menu[1].item[2].type = submenu;
    menu[1].item[2].value = 0;
    menu[1].item[2].subIndex = 4;
    menu[1].item[2].function = NULL;
    
    strcpy(menu[2].title, "STAND");
    menu[2].count = 4;
    menu[2].parIndex = 1;
    strcpy(menu[2].item[0].name, "P");
    menu[2].item[0].type = dec;
    menu[2].item[0].value = 0;
    menu[2].item[0].subIndex = -1;
    menu[2].item[0].function = NULL;
    strcpy(menu[2].item[1].name, "I");
    menu[2].item[1].type = dec;
    menu[2].item[1].value = 0;
    menu[2].item[1].subIndex = -1;
    menu[2].item[1].function = NULL;
    strcpy(menu[2].item[2].name, "D");
    menu[2].item[2].type = dec;
    menu[2].item[2].value = 0;
    menu[2].item[2].subIndex = -1;
    menu[2].item[2].function = NULL;
    strcpy(menu[2].item[3].name, "OK");
    menu[2].item[3].type = function;
    menu[2].item[3].value = 0;
    menu[2].item[3].subIndex = -1;
    menu[2].item[3].function = Flash;
    
    strcpy(menu[3].title, "DIR");
    menu[3].count = 4;
    menu[3].parIndex = 1;
    strcpy(menu[3].item[0].name, "P");
    menu[3].item[0].type = dec;
    menu[3].item[0].value = 0;
    menu[3].item[0].subIndex = -1;
    menu[3].item[0].function = NULL;
    strcpy(menu[3].item[1].name, "I");
    menu[3].item[1].type = dec;
    menu[3].item[1].value = 0;
    menu[3].item[1].subIndex = -1;
    menu[3].item[1].function = NULL;
    strcpy(menu[3].item[2].name, "D");
    menu[3].item[2].type = dec;
    menu[3].item[2].value = 0;
    menu[3].item[2].subIndex = -1;
    menu[3].item[2].function = NULL;
    strcpy(menu[3].item[3].name, "OK");
    menu[3].item[3].type = function;
    menu[3].item[3].value = 0;
    menu[3].item[3].subIndex = -1;
    menu[3].item[3].function = Flash;
    
    strcpy(menu[4].title, "STAND");
    menu[4].count = 4;
    menu[4].parIndex = 1;
    strcpy(menu[4].item[0].name, "P");
    menu[4].item[0].type = dec;
    menu[4].item[0].value = 0;
    menu[4].item[0].subIndex = -1;
    menu[4].item[0].function = NULL;
    strcpy(menu[4].item[1].name, "I");
    menu[4].item[1].type = dec;
    menu[4].item[1].value = 0;
    menu[4].item[1].subIndex = -1;
    menu[4].item[1].function = NULL;
    strcpy(menu[4].item[2].name, "D");
    menu[4].item[2].type = dec;
    menu[4].item[2].value = 0;
    menu[4].item[2].subIndex = -1;
    menu[4].item[2].function = NULL;
    strcpy(menu[4].item[3].name, "OK");
    menu[4].item[3].type = function;
    menu[4].item[3].value = 0;
    menu[4].item[3].subIndex = -1;
    menu[4].item[3].function = Flash;
    
    strcpy(menu[5].title, "D");
    menu[5].count = 2;
    menu[5].parIndex = 0;
    strcpy(menu[5].item[0].name, "Start");
    menu[5].item[0].type = function;
    menu[5].item[0].value = 0;
    menu[5].item[0].subIndex = -1;
    menu[5].item[0].function = D;
    strcpy(menu[5].item[1].name, "Save");
    menu[5].item[1].type = function;
    menu[5].item[1].value = 0;
    menu[5].item[1].subIndex = -1;
    menu[5].item[1].function = Save;
}

int8_t menuIndex = 0,itemIndex = 0,mode = 0;

void OLED_ShowMenu(void)
{   
        
        oled_show_int(108,0,Mission,1);
	oled_show_string(0,0,menu[menuIndex].title);
	for(int16_t i = 0;i < menu[menuIndex].count;i++)
	{
            if(i == itemIndex){
		oled_show_string(0,(i + 1),">");
            }
            oled_show_string(6,(i + 1),menu[menuIndex].item[i].name);

            if(menu[menuIndex].item[i].type == dec )
            {
		oled_show_float(72,(i + 1),menu[menuIndex].item[i].value,1,3);
            }
	}
        
	
	
}


void up(void)
{       
        oled_clear();
        if(mode == 0){
          itemIndex--;
          if(itemIndex < 0){
            itemIndex = menu[menuIndex].count-1;
          }
        } else {
          menu[menuIndex].item[itemIndex].value += 0.005;
        }
}


void down(void)
{
        oled_clear();
        if(mode == 0){
          itemIndex++;
          if(itemIndex == menu[menuIndex].count){
            itemIndex = 0;
          }
        } else {
          menu[menuIndex].item[itemIndex].value -= 0.005;
        }
	
}

void back(void)
{
        oled_clear();
        if(mode == 1 && menu[menuIndex].item[itemIndex].type == dec)
	{
		mode = 0;
	}else if(menu[menuIndex].parIndex != -1 && mode == 0){
		itemIndex = 0;
		menuIndex = menu[menuIndex].parIndex;
	}
}

void confirm(void)
{
        oled_clear();
        if(menu[menuIndex].item[itemIndex].type == function){
           menu[menuIndex].item[itemIndex].function();
        } else 
	if(menu[menuIndex].item[itemIndex].subIndex != -1 && menu[menuIndex].item[itemIndex].type != dec){
		menuIndex = menu[menuIndex].item[itemIndex].subIndex;
		itemIndex = 0;
	} else if(mode == 0 && menu[menuIndex].item[itemIndex].type == dec)
	{
		mode = 1;
	} else if( mode == 1 && menu[menuIndex].item[itemIndex].type == dec)
	{
		mode = 0;
	}
	
}
void Key_Scanner(void)
{
        key_scanner(); 
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS){
          down();
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS){
          up();
        }  
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS){
          back();
        }  
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS){
          confirm();
        }  
  
}
