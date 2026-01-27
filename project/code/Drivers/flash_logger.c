#include "zf_common_headfile.h"
#include "flash_logger.h"

/**
 * @brief 初始化Flash日志记录器
 */
void Logger_Init(void)
{
    flash_init();
}

/**
 * @brief 向Flash日志记录器写入数据块
 *
 * @param start_page 起始页码
 * @param buffer 数据缓冲区指针
 * @param byte_len 要写入的字节长度
 */
void Logger_WriteBlock(uint32 start_page, void *buffer, uint32 byte_len)
{
    uint32 *p_src = (uint32 *)buffer;
    uint32 totol_words = (byte_len + 3) / 4; // 向上取整为字数
    uint32 words_per_page = FLASH_PAGE_SIZE / 4;
    uint32 pages_needed = (totol_words + words_per_page - 1) / words_per_page; // 向上取整为页数

    for (uint32 i = 0; i < pages_needed; i++)
    {
        uint32 current_page = start_page + i;
        uint32 words_to_write = (totol_words > words_per_page) ? words_per_page : totol_words;

        if (words_to_write == 0)
        {
            break;
        }

        flash_write_page(0, current_page, p_src, words_to_write);
        p_src += words_to_write;
        totol_words -= words_to_write;
    }
}

/**
 * @brief 从Flash日志记录器读取数据块
 *
 * @param start_page 起始页码
 * @param buffer 数据缓冲区指针
 * @param byte_len 要读取的字节长度
 */
void Logger_ReadBlock(uint32 start_page, void *buffer, uint32 byte_len)
{
    uint32 *p_dest = (uint32 *)buffer;
    uint32 totol_words = (byte_len + 3) / 4; // 向上取整为字数
    uint32 words_per_page = FLASH_PAGE_SIZE / 4;
    uint32 pages_needed = (byte_len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE; // 向上取整为页数

    for (uint32 i = 0; i < pages_needed; i++)
    {
        uint32 current_page = start_page + i;
        uint32 words_to_read = (totol_words > words_per_page) ? words_per_page : totol_words;

        if (words_to_read == 0)
        {
            break;
        }

        flash_read_page(0, current_page, p_dest, words_to_read);
        p_dest += words_to_read;
        totol_words -= words_to_read;
    }
}