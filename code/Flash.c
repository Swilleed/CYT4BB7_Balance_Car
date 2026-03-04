#include "zf_common_typedef.h"
#include "zf_driver_flash.h"
#include <string.h>
#include "Flash.h"



uint16_t flashcount1 = 0;
uint8_t flash_page_index = 0;
uint16_t flashcount2 = 0;
uint8_t read_page_index = 0;
uint8_t start_flag;



void flash_save(void)
{
  if(flash_check(0, flash_page_index)){
     flash_erase_page(0, flash_page_index);
  }
  flash_write_page_from_buffer(0,flash_page_index,500);
  memset(flash_union_buffer,0,sizeof(flash_union_buffer));
  flashcount1 = 0;
  flash_page_index ++;
}


void buffer_write_flash(Speed_Data Data)
{
  flash_union_buffer[flashcount1].float_type = Data.Speed;
  flash_union_buffer[flashcount1 + 250].float_type = Data.Yaw;
  flashcount1 ++;
  if(flashcount1 >= 250){
    flash_save();
  }
}

uint8_t last_state = 0;
Speed_Data Stop = {
  .Speed = 200,
  .Yaw = 200,
};
  
void Flash_scanner(uint8_t state,Speed_Data Data){
     if(state){
        buffer_write_flash(Data);
    } else if(last_state && !state){
        flash_union_buffer[flashcount1].float_type = Stop.Speed;
        flash_union_buffer[flashcount1 + 250].float_type = Stop.Yaw;
        flashcount1 ++;
        flash_save();
        
    }    
    last_state = state;
}

void buffer_read_flash(Speed_Data *Data){
  if(start_flag == read_page_index){
    flash_read_page_to_buffer(0,read_page_index,500);
    start_flag ++;
  }
  if(flash_union_buffer[flashcount2 + 250].float_type != 200){
    
    Data->Speed = flash_union_buffer[flashcount2].float_type;
    Data->Yaw = flash_union_buffer[flashcount2 + 250].float_type;
  }
  flashcount2 ++;
  if(flashcount2 >= 250){
    flashcount2 = 0;
    read_page_index ++;
  }
}

