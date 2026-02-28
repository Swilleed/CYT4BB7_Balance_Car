#include "zf_common_headfile.h"
#include "Bluetooth.h"
#include <stdlib.h>
void Bluetooth_receive(uint8 data_buffer[],uint8 data_len,int *Data1,int *Data2){
  if(data_len != 0){
    char temp[2][6];  
    int i = 0, j = 0;
    for (i = 0; (i < 5) && (10 + i < data_len) && data_buffer[10 + i] != ','; i++) {
        temp[0][i] = (char)data_buffer[10 + i];
    }
    temp[0][i] = '\0';  
    for (j = 0; (j < 5) && (11 + i + j < data_len) && data_buffer[11 + i + j] != ','; j++) {
        temp[1][j] = (char)data_buffer[11 + i + j];
    }
    temp[1][j] = '\0';  
    
    *Data1 = atoi(temp[0]);
    *Data2 = atoi(temp[1]);
    
    memset(data_buffer, 0, 32);
  }
}

