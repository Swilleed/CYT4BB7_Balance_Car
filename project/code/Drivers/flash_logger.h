#ifndef _FLASH_LOGGER_H_
#define _FLASH_LOGGER_H_

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 256 // 假设每页大小为256字节，根据实际
#endif

void Logger_Init(void);
void Logger_WriteBlock(uint32 start_page, void *buffer, uint32 byte_len);
void Logger_ReadBlock(uint32 start_page, void *buffer, uint32 byte_len);
void Logger_Debug_Test(void);

#endif /* _FLASH_LOGGER_H_ */
