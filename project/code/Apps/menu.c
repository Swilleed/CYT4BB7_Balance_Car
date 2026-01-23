#include "zf_common_headfile.h"
#include "menu.h"

Menu *CurrentMenu;
uint8_t CurrentSelection = 0;

static Menu MainMenu = {
    .title = "Main Menu",
    .parent = NULL,
    .children = {NULL},
    .child_count = 0,
    .action = NULL,
};

Menu *InitMenu(void)
{
    // 初始化主菜单
    CurrentMenu = &MainMenu;
    CurrentSelection = 0;
    return CurrentMenu;
}

/**
 * 辅助函数，将浮点数转换为字符串
 * @param value 浮点数值
 * @param buffer 存储字符串的缓冲区
 * @param bufferSize 缓冲区大小
 * @return 目标字符串
 */
static char *FloatToCharArray(float value, char *buffer, int bufferSize)
{
    snprintf(buffer, bufferSize, "%.2f", value);
    return buffer;
}

// 进入子菜单
static void NavigateToChild(void)
{
    if (CurrentSelection < CurrentMenu->child_count)
    {
        CurrentMenu = CurrentMenu->children[CurrentSelection];
        CurrentSelection = 0;
    }
}

// 返回上级菜单
static void NavigateToParent(void)
{
    if (CurrentMenu->parent != NULL)
    {
        CurrentMenu = CurrentMenu->parent;
        CurrentSelection = 0;
    }
}

static void HandleInput(void)
{
    // 有子菜单：浏览/进入/返回
    if (CurrentMenu->child_count > 0)
    {
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS)
        {
            if (CurrentSelection < CurrentMenu->child_count - 1)
            {
                CurrentSelection++; // 下移
            }
            else if (CurrentSelection == CurrentMenu->child_count - 1)
            {
                CurrentSelection = 0;
            }
        }
        else if (key_get_state(KEY_UP) == KEY_SHORT_PRESS)
        {
            if (CurrentSelection > 0)
            {
                CurrentSelection--; // 上移
            }
            else if (CurrentSelection == 0)
            {
                CurrentSelection = CurrentMenu->child_count - 1;
            }
        }
        else if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS)
        {
            NavigateToChild(); // 进入子菜单
        }
        else if (CurrentMenu->parent != NULL && key_get_state(KEY_BACK) == KEY_SHORT_PRESS)
        {
            NavigateToParent(); // 返回上级菜单
        }
    }
    // 叶子菜单：短按返回
    else
    {
        if (key_get_state(KEY_BACK) == KEY_SHORT_PRESS)
        {
            NavigateToParent();
        }
        //     else if (Key_Check(KEY_1, KEY_LONG))
        //     {
        //         SaveADCToFlash();
        //     }
    }
}

void DisplayMenu(void)
{
    // 先处理按键，确保本次渲染使用最新选择状态
    HandleInput();

    // 叶子节点：只展示动作页面
    if (CurrentMenu->child_count == 0)
    {
        if (CurrentMenu->action != NULL)
        {
            CurrentMenu->action();
        }
        return;
    }

    oled_clear_screen();
    oled_show_string(1, 1, "Main");

    for (uint8_t i = 0; i < CurrentMenu->child_count; i++)
    {
        uint8_t row = i + 2; // 行号从1开始，第二行起列出子菜单
        char line[17];
        snprintf(line, sizeof(line), "%u %c%s", (unsigned)(i + 1), (i == CurrentSelection) ? '>' : ' ', CurrentMenu->children[i]->title);
        oled_show_string(row, 1, line);
    }
}
