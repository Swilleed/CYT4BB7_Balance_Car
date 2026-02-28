#include "Infrared.h"


uint8_t Infrared[5] = {0};

void Infrared_Init(void)
{
  gpio_init(P03_0, GPI, 0, GPI_PULL_DOWN);
  gpio_init(P03_1, GPI, 0, GPI_PULL_DOWN);
  gpio_init(P03_2, GPI, 0, GPI_PULL_DOWN);
  gpio_init(P03_3, GPI, 0, GPI_PULL_DOWN);
  gpio_init(P03_4, GPI, 0, GPI_PULL_DOWN);
}

int8_t Infrared_ErrorGet(uint8_t *Infrared)
{ 
   Infrared[0] = gpio_get_level (P03_0);
   Infrared[1] = gpio_get_level (P03_1); 
   Infrared[2] = gpio_get_level (P03_2);
   Infrared[3] = gpio_get_level (P03_3);
   Infrared[4] = gpio_get_level (P03_4);
   
   return Infrared[0] * 7 + Infrared[1] * 3 + Infrared[3] * -3 + Infrared[4] * -7;
   
   
}