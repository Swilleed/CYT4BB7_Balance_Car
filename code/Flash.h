#ifndef __FLASH_H_
#define __FLASH_H_

typedef struct{
  float Speed;
  float Yaw;
}Speed_Data;

void flash_save(void);
void buffer_write_flash(Speed_Data Data);
void Flash_scanner(uint8_t state,Speed_Data Data);
void buffer_read_flash(Speed_Data *Data);

#endif