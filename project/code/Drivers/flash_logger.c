#include "zf_common_headfile.h"
#include "flash_logger.h"
#include "zf_device_oled.h"

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

/**
 * @brief Flash Logger 功能自测调试接口
 * @note 该函数会测试从第 60 页开始存取 3 个 PathPoint_t 结构体
 */
void Logger_Debug_Test(void)
{
    // 1. 准备测试数据
    typedef struct
    {
        int16 left_ticks;
        int16 right_ticks;
        float yaw;
    } TestPoint_t; // 临时定义一个结构体，模拟应用层数据

    TestPoint_t write_data[3] = {
        {100, -100, 12.5f},
        {500, 200, 45.0f},
        {999, 888, 179.9f}};
    TestPoint_t read_data[3] = {0}; // 接收读取结果

    oled_clear();
    oled_show_string(0, 0, "Flash Test Inc");

    // 2. 写入数据到 Flash (从第 60 页开始)
    Logger_WriteBlock(60, (void *)write_data, sizeof(write_data));
    oled_show_string(0, 1, "Write Done");

    // 3. 读回数据
    Logger_ReadBlock(60, (void *)read_data, sizeof(read_data));
    oled_show_string(0, 2, "Read Done");

    // 4. 显示读回的数据进行比对 (取第 1 个点和第 3 个点)
    // 显示第 1 个点的 Left Ticks (预期 100)
    oled_show_string(0, 3, "P1 L:");
    oled_show_int(40, 3, read_data[0].left_ticks, 4);

    // 显示第 3 个点的 Yaw (预期 179.9)
    oled_show_string(0, 4, "P3 Y:");
    oled_show_float(40, 4, read_data[2].yaw, 3, 1);

    // 5. 自动校验
    if (memcmp(write_data, read_data, sizeof(write_data)) == 0)
    {
        oled_show_string(0, 6, "VERIFY: SUCCESS");
    }
    else
    {
        oled_show_string(0, 6, "VERIFY: FAIL!!!");
    }
}