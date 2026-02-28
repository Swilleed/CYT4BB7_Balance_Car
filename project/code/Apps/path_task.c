#include "path_task.h"
#include "odometry.h"
#include "flash_logger.h"
#include "balance_task.h"
#include <math.h>
#include <string.h>

/**
 * path_task 模块操作说明（示教复现）
 *
 * 一、模块目标
 * 1) 录制阶段：按固定周期把 current_pose 轨迹点写入 RAM 缓冲。
 * 2) 保存阶段：把“路径头 + 有效节点数据”写入 Flash。
 * 3) 回放阶段：从 Flash 恢复有效路径，逐点调用 Balance_Chase_Position 追逐。
 *
 * 二、推荐调用时序（由上层状态机驱动）
 * 1) 上电后调用 Path_Init()。
 * 2) 进入录制：调用 Path_Start_Record()。
 * 3) 录制中：每 20ms 调用 Path_Record_Tick()。
 * 4) 停止录制：调用 Path_Stop_Record()，自动落盘到 Flash。
 * 5) 进入回放：调用 Path_Start_Playback()，自动校验并装载路径。
 * 6) 回放中：每 20ms 调用 Path_Playback_Tick()。
 * 7) 结束/中断：调用 Path_Stop_Playback()。
 *
 * 三、Flash 数据布局
 * 1) PATH_FLASH_PAGE：保存路径头 PathFlashHeader_t（magic + count）。
 * 2) 后续页：仅保存有效节点数 count 对应的数据（非整块 MAX_PATH_NODES）。
 *
 * 四、回放判定规则
 * 1) 每次取当前 playback_index 目标点。
 * 2) 调用 Balance_Chase_Position(target.x, target.y, target.yaw)。
 * 3) 若返回 1（到点），playback_index++，继续追下一个点。
 * 4) 所有点追完后自动转入 PATH_IDLE。
 *
 * 五、注意事项
 * 1) 本模块依赖上层已正确更新里程计 current_pose。
 * 2) 若 magic/count 校验失败，会拒绝回放并保持 PATH_IDLE。
 * 3) 录制频率建议保持 20ms，过快会导致点过密、Flash 压力增加。
 */

PathState_enum path_state = PATH_IDLE;
uint16 path_node_count = 0;
static uint16 playback_index = 0;
static PathNode_t path_buffer[MAX_PATH_NODES];

typedef struct
{
    uint32 magic;
    uint16 count;
    uint16 reserved;
} PathFlashHeader_t;

/**
 * @return 计算得到的数据区起始页地址
 * 计算路径数据在 Flash 的起始页
 * 为什么这样写：路径头与路径点分开存储，便于先校验有效性再读取数据。
 * 怎么实现：按头结构体大小折算占用页数，数据区从 PATH_FLASH_PAGE 后续页开始。
 * 怎么调用：在 Path_Stop_Record 写数据和 Path_Start_Playback 读数据时调用。
 * 对应效果：保证头与数据不覆盖，读写边界明确。
 */
static uint32 Path_DataStartPage(void)
{
    uint32 header_pages = (sizeof(PathFlashHeader_t) + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE;
    return (PATH_FLASH_PAGE + header_pages);
}

/**
 * 初始化路径任务模块
 * 为什么这样写：上电或重置时需要把状态与缓存归零，避免沿用脏数据。
 * 怎么实现：状态置 PATH_IDLE，节点计数清零，路径缓存 memset 清空。
 * 怎么调用：建议系统初始化阶段调用一次。
 * 对应效果：路径模块进入可预期初始态。
 */
void Path_Init(void)
{
    path_state = PATH_IDLE;
    path_node_count = 0;
    memset(path_buffer, 0, sizeof(path_buffer));
}

/**
 * 开始路径录制
 * 为什么这样写：录制必须从零开始，且坐标系要与录制起点对齐。
 * 怎么实现：清零 path_node_count，调用 Odometry_Reset，将状态设为 PATH_RECORDING。
 * 怎么调用：示教模式进入录制时调用一次。
 * 对应效果：后续记录点相对同一起点，便于稳定回放。
 */
void Path_Start_Record(void)
{
    path_node_count = 0;
    Odometry_Reset();
    path_state = PATH_RECORDING;
}

/**
 * 停止路径录制并保存到 Flash
 * 为什么这样写：需要把“有效点数”持久化，否则重启后无法知道有效路径长度。
 * 怎么实现：
 * 1) 写入路径头（magic + count）；
 * 2) 若 count>0，仅写入有效节点数据；
 * 3) 保存完成后状态回 PATH_IDLE。
 * 怎么调用：录制结束时调用一次。
 * 对应效果：路径可跨重启复现，且不会读取无效尾部数据。
 */
void Path_Stop_Record(void)
{
    if (path_state == PATH_RECORDING)
    {
        PathFlashHeader_t header;

        path_state = PATH_SAVING;

        header.magic = PATH_FLASH_MAGIC;
        header.count = path_node_count;
        header.reserved = 0;

        Logger_WriteBlock(PATH_FLASH_PAGE, (void *)&header, sizeof(header));

        if (path_node_count > 0)
        {
            Logger_WriteBlock(Path_DataStartPage(), (void *)path_buffer, (uint32)(path_node_count * sizeof(PathNode_t)));
        }

        path_state = PATH_IDLE;
    }
}

/**
 * 开始路径复现
 * 为什么这样写：回放前必须确认 Flash 中确实有合法路径数据。
 * 怎么实现：
 * 1) 读取路径头并校验 magic/count；
 * 2) 校验通过后按 count 读取有效节点；
 * 3) playback_index 归零、里程计复位，状态切换到 PATH_PLAYBACK。
 * 怎么调用：进入复现模式时调用一次。
 * 对应效果：非法数据会被拦截，合法数据可立即进入逐点追踪。
 */
void Path_Start_Playback(void)
{
    PathFlashHeader_t header;

    memset(&header, 0, sizeof(header));
    Logger_ReadBlock(PATH_FLASH_PAGE, (void *)&header, sizeof(header));

    if ((header.magic != PATH_FLASH_MAGIC) || (header.count == 0) || (header.count > MAX_PATH_NODES))
    {
        path_node_count = 0;
        playback_index = 0;
        path_state = PATH_IDLE;
        return;
    }

    path_node_count = header.count;
    memset(path_buffer, 0, sizeof(path_buffer));
    Logger_ReadBlock(Path_DataStartPage(), (void *)path_buffer, (uint32)(path_node_count * sizeof(PathNode_t)));

    playback_index = 0;
    Odometry_Reset();
    path_state = PATH_PLAYBACK;
}

/**
 * 停止路径复现
 * 为什么这样写：提供统一的回放终止出口，便于上层中断或自动结束。
 * 怎么实现：将状态置为 PATH_IDLE。
 * 怎么调用：用户中断、点追完、异常退出时调用。
 * 对应效果：路径模块停止输出追逐动作，回到空闲态。
 */
void Path_Stop_Playback(void)
{
    path_state = PATH_IDLE;
}

/**
 * 录制一帧数据 (建议 20ms 调用一次)
 * 为什么这样写：轨迹用离散点表示，固定周期采样可在精度与存储量之间平衡。
 * 怎么实现：
 * 1) 仅在 PATH_RECORDING 状态生效；
 * 2) 将 current_pose 的 x/y/yaw 复制到 path_buffer；
 * 3) 超出容量自动停止录制并保存。
 * 怎么调用：由上层 20ms 调度调用。
 * 对应效果：录制得到连续轨迹点序列，可用于后续复现。
 */
void Path_Record_Tick(void)
{
    if (path_state != PATH_RECORDING)
        return;

    if (path_node_count < MAX_PATH_NODES)
    {
        path_buffer[path_node_count].x = current_pose.x;
        path_buffer[path_node_count].y = current_pose.y;
        path_buffer[path_node_count].yaw = current_pose.yaw;
        path_node_count++;
    }
    else
    {
        Path_Stop_Record();
    }
}

/**
 * 路径复现处理 (建议 20ms 调用一次)
 * 为什么这样写：把“取目标点 + 追逐 + 到点推进”封装在同一处，保持上层简单。
 * 怎么实现：
 * 1) 检查状态与索引边界；
 * 2) 取当前目标点 target；
 * 3) 调用 Balance_Chase_Position 执行追逐；
 * 4) 到点后 playback_index++，继续下一个点。
 * 怎么调用：由上层 20ms 调度持续调用直到状态退出 PATH_PLAYBACK。
 * 对应效果：小车按录制路径逐点复现，追完自动结束。
 */
void Path_Playback_Tick(void)
{
    if (path_state != PATH_PLAYBACK)
        return;

    if (playback_index >= path_node_count)
    {
        Path_Stop_Playback();
        return;
    }

    PathNode_t *target = &path_buffer[playback_index];

    if (Balance_Chase_Position(target->x, target->y, target->yaw))
    {
        playback_index++;
    }
}
