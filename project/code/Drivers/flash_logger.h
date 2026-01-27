#ifndef _FLASH_LOGGER_H_
#define _FLASH_LOGGER_H_

void Logger_Init(void);
void Logger_WriteBlock(uint32 start_page, void *buffer, uint32 byte_len);
void Logger_ReadBlock(uint32 start_page, void *buffer, uint32 byte_len);
void Logger_Debug_Test(void);

#endif /* _FLASH_LOGGER_H_ */
