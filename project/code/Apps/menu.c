#include "zf_common_headfile.h"
#include "menu.h"


// 直接extern需要的变量
extern PID_TypeDef pid_pitch_out;
extern PID_TypeDef pid_gyro_in;
extern LineFollower_t follower;
extern float MADGWICK_BETA;

// 运动参数 - 简单命名
float pos_p = 0.6f;          
float pos_i = 0.1f;          
float pos_d = 0.0f;          

uint8_t spd = 40;             // 基础速度
int16_t off = 0;              // 陀螺仪零偏
float gain = 1.0f;            // 转向增益
float flt = 0.7f;             // 低通滤波系数
uint8_t flt_dep = 5;          // 数字滤波深度

// 模式计数变量
uint16_t lap = 0;             // 圈数
uint16_t cyc = 0;             // 循环数
uint16_t pt = 0;              // 记录点数

// 全局菜单变量
Menu *CurrentMenu;
uint8_t CurrentSelection = 0;
static uint8_t edit_mode = 0;  // 0:浏览模式 1:编辑模式
static uint8_t edit_param = 0;  // 当前编辑的参数索引

// 菜单变量声明
static Menu MainMenu;
static Menu PIDMenu, MOVEMenu, MODEMenu;
static Menu ANGLEMenu, GYROMenu, TRACKMenu, POSMenu;
static Menu Mode1Menu, Mode2Menu, Mode3Menu, Mode4Menu, Mode5Menu;
static Menu Mode2P1Menu, Mode2P2Menu, Mode3P1Menu, Mode3P2Menu, Mode4PMenu;

/*============================================================================
 * Flash参数保存（结构体方式）
 *============================================================================*/
#define PARAM_FLASH_PAGE 500

typedef struct {
    // PID参数
    float angle_p, angle_i, angle_d;
    float gyro_p, gyro_i, gyro_d;
    float track_p, track_i, track_d;
    
    // 位置环参数
    float pos_p, pos_i, pos_d;
    
    // 运动参数
    uint8_t spd;
    int16_t off;
    float gain, flt;
    uint8_t flt_dep;
    
    // 其他
    float madgwick_beta;
    uint16_t magic;
} ParamSave_t;

static void SaveParams(void)
{
    ParamSave_t param;
    
    // PID参数
    param.angle_p = pid_pitch_out.Kp;
    param.angle_i = pid_pitch_out.Ki;
    param.angle_d = pid_pitch_out.Kd;
    
    param.gyro_p = pid_gyro_in.Kp;
    param.gyro_i = pid_gyro_in.Ki;
    param.gyro_d = pid_gyro_in.Kd;
    
    param.track_p = follower.pid.Kp;
    param.track_i = follower.pid.Ki;
    param.track_d = follower.pid.Kd;
    
    // 位置环
    param.pos_p = pos_p;
    param.pos_i = pos_i;
    param.pos_d = pos_d;
    
    // 运动参数
    param.spd = spd;
    param.off = off;
    param.gain = gain;
    param.flt = flt;
    param.flt_dep = flt_dep;
    
    param.madgwick_beta = MADGWICK_BETA;
    param.magic = 0x5A5A;
    
    Logger_WriteBlock(PARAM_FLASH_PAGE, &param, sizeof(ParamSave_t));
}

static void LoadParams(void)
{
    ParamSave_t param;
    Logger_ReadBlock(PARAM_FLASH_PAGE, &param, sizeof(ParamSave_t));
    
    if (param.magic == 0x5A5A) {
        // PID参数
        pid_pitch_out.Kp = param.angle_p;
        pid_pitch_out.Ki = param.angle_i;
        pid_pitch_out.Kd = param.angle_d;
        
        pid_gyro_in.Kp = param.gyro_p;
        pid_gyro_in.Ki = param.gyro_i;
        pid_gyro_in.Kd = param.gyro_d;
        
        follower.pid.Kp = param.track_p;
        follower.pid.Ki = param.track_i;
        follower.pid.Kd = param.track_d;
        
        // 位置环
        pos_p = param.pos_p;
        pos_i = param.pos_i;
        pos_d = param.pos_d;
        
        // 运动参数
        spd = param.spd;
        off = param.off;
        gain = param.gain;
        flt = param.flt;
        flt_dep = param.flt_dep;
        
        MADGWICK_BETA = param.madgwick_beta;
    }
}

/*============================================================================
 * 参数编辑函数
 *============================================================================*/
static void EditFloat(float *val, float min, float max, float step)
{
    if (key_get_state(KEY_UP) == KEY_SHORT_PRESS) {
        *val += step;
        if (*val > max) *val = max;
    }
    if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS) {
        *val -= step;
        if (*val < min) *val = min;
    }
}

static void EditInt(uint8_t *val, uint8_t min, uint8_t max, uint8_t step)
{
    if (key_get_state(KEY_UP) == KEY_SHORT_PRESS) {
        *val += step;
        if (*val > max) *val = max;
    }
    if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS) {
        *val -= step;
        if (*val < min) *val = min;
    }
}


static Menu MainMenu = {
    .title = "Main Menu",
    .parent = NULL,
    .children = {NULL},
    .child_count = 0,
    .action = NULL,
};

Menu *InitMenu(void)
{
    LoadParams();  // 加载保存的参数
    
    MainMenu.title = "Main Menu";
    
    PIDMenu.title = "PID";
    MOVEMenu.title = "MOVE";
    MODEMenu.title = "MODE";
    
    ANGLEMenu.title = "ANGLE";
    GYROMenu.title = "GYRO";
    TRACKMenu.title = "TRACK";
    POSMenu.title = "POS";
    
    Mode1Menu.title = "1";
    Mode2Menu.title = "2";
    Mode3Menu.title = "3";
    Mode4Menu.title = "4";
    Mode5Menu.title = "5";
    
    Mode2P1Menu.title = "path1";
    Mode2P2Menu.title = "path2";
    Mode3P1Menu.title = "path1";
    Mode3P2Menu.title = "path2";
    Mode4PMenu.title = "path";
    
    PIDMenu.action = NULL;
    MOVEMenu.action = MOVE_Action;
    MODEMenu.action = NULL;
    
    ANGLEMenu.action = ANGLE_Action;
    GYROMenu.action = GYRO_Action;
    TRACKMenu.action = TRACK_Action;
    POSMenu.action = POS_Action;
    
    Mode1Menu.action = M1_Action;
    Mode2Menu.action = M2_Action;
    Mode3Menu.action = M3_Action;
    Mode4Menu.action = M4_Action;
    Mode5Menu.action = M5_Action;
    
    Mode2P1Menu.action = M2P1_Action;
    Mode2P2Menu.action = M2P2_Action;
    Mode3P1Menu.action = M3P1_Action;
    Mode3P2Menu.action = M3P2_Action;
    Mode4PMenu.action = M4P_Action;
    
    Menu_AddChild(&MainMenu, &PIDMenu);
    Menu_AddChild(&MainMenu, &MOVEMenu);
    Menu_AddChild(&MainMenu, &MODEMenu);
    
    Menu_AddChild(&PIDMenu, &ANGLEMenu);
    Menu_AddChild(&PIDMenu, &GYROMenu);
    Menu_AddChild(&PIDMenu, &TRACKMenu);
    Menu_AddChild(&PIDMenu, &POSMenu);
    
    Menu_AddChild(&MODEMenu, &Mode1Menu);
    Menu_AddChild(&MODEMenu, &Mode2Menu);
    Menu_AddChild(&MODEMenu, &Mode3Menu);
    Menu_AddChild(&MODEMenu, &Mode4Menu);
    Menu_AddChild(&MODEMenu, &Mode5Menu);
    
    Menu_AddChild(&Mode2Menu, &Mode2P1Menu);
    Menu_AddChild(&Mode2Menu, &Mode2P2Menu);
    Menu_AddChild(&Mode3Menu, &Mode3P1Menu);
    Menu_AddChild(&Mode3Menu, &Mode3P2Menu);
    Menu_AddChild(&Mode4Menu, &Mode4PMenu);

    // 初始化主菜单
    CurrentMenu = &MainMenu;
    CurrentSelection = 0;
    return CurrentMenu;
}

/**
 * 添加子菜单项
 * @param parent 父菜单指针
 * @param child 子菜单指针
 */
void Menu_AddChild(Menu *parent, Menu *child)
{
    if (parent->child_count < MENU_MAX_CHILDREN)
    {
        parent->children[parent->child_count] = child;
        parent->child_count++;
        child->parent = parent;
    }
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
            CurrentSelection = (CurrentSelection + 1) % CurrentMenu->child_count;
            /*
            if (CurrentSelection < CurrentMenu->child_count - 1)
            {
                CurrentSelection++; // 下移
            }
            else if (CurrentSelection == CurrentMenu->child_count - 1)
            {
                CurrentSelection = 0;
            }
            */
        }
        else if (key_get_state(KEY_UP) == KEY_SHORT_PRESS)
        {
            CurrentSelection = (CurrentSelection + CurrentMenu->child_count - 1) % CurrentMenu->child_count;
            /*
            if (CurrentSelection > 0)
            {
                CurrentSelection--; // 上移
            }
            else if (CurrentSelection == 0)
            {
                CurrentSelection = CurrentMenu->child_count - 1;
            }
            */
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

    // oled_clear();
    //oled_show_string(1, 1, CurrentMenu->title);

    for (uint8_t i = 0; i < CurrentMenu->child_count; i++)
    {
        uint8_t row = i + 2; // 行号从1开始，第二行起列出子菜单
        char line[17];
        snprintf(line, sizeof(line), "%u %c%s", (unsigned)(i + 1), (i == CurrentSelection) ? '>' : ' ', CurrentMenu->children[i]->title);
        oled_show_string(1, row, line);
    }
}

/*============================================================================
 * 叶子菜单的Action函数
 *============================================================================*/

// 1.1 角度环
static void ANGLE_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "ANGLE");
    
    char line[17];
    sprintf(line, "P:%.1f", pid_pitch_out.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", pid_pitch_out.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", pid_pitch_out.Kd);
    oled_show_string(1, 5, line);
    
    if (edit_mode) {
        if (edit_param == 0) EditFloat(&pid_pitch_out.Kp, 0, 50, 1);
        if (edit_param == 1) EditFloat(&pid_pitch_out.Ki, 0, 10, 0.1);
        if (edit_param == 2) EditFloat(&pid_pitch_out.Kd, 0, 10, 0.1);
    }
}

// 1.2 角速度环
static void GYRO_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "GYRO");
    
    char line[17];
    sprintf(line, "P:%.1f", pid_gyro_in.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", pid_gyro_in.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", pid_gyro_in.Kd);
    oled_show_string(1, 5, line);
    
    if (edit_mode) {
        if (edit_param == 0) EditFloat(&pid_gyro_in.Kp, 0, 50, 1);
        if (edit_param == 1) EditFloat(&pid_gyro_in.Ki, 0, 10, 0.1);
        if (edit_param == 2) EditFloat(&pid_gyro_in.Kd, 0, 10, 0.1);
    }
}

// 1.3 循迹环
static void TRACK_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "TRACK");
    
    char line[17];
    sprintf(line, "P:%.1f", follower.pid.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", follower.pid.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", follower.pid.Kd);
    oled_show_string(1, 5, line);
    
    if (edit_mode) {
        if (edit_param == 0) EditFloat(&follower.pid.Kp, 0, 50, 1);
        if (edit_param == 1) EditFloat(&follower.pid.Ki, 0, 10, 0.1);
        if (edit_param == 2) EditFloat(&follower.pid.Kd, 0, 20, 0.5);
    }
}

// 1.4 位置环
static void POS_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "POS");
    
    char line[17];
    sprintf(line, "P:%.2f", pos_p);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", pos_i);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", pos_d);
    oled_show_string(1, 5, line);
    
    if (edit_mode) {
        if (edit_param == 0) EditFloat(&pos_p, 0, 2, 0.05);
        if (edit_param == 1) EditFloat(&pos_i, 0, 1, 0.01);
        if (edit_param == 2) EditFloat(&pos_d, 0, 1, 0.01);
    }
}

// 2 MOVE
static void MOVE_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "MOVE");
    
    char line[17];
    sprintf(line, "spd:%d", spd);
    oled_show_string(1, 3, line);
    sprintf(line, "off:%d", off);
    oled_show_string(1, 4, line);
    sprintf(line, "gain:%.1f", gain);
    oled_show_string(1, 5, line);
    sprintf(line, "flt:%.1f", flt);
    oled_show_string(1, 6, line);
    sprintf(line, "beta:%.2f", MADGWICK_BETA);
    oled_show_string(1, 7, line);
    
    if (edit_mode) {
        if (edit_param == 0) EditInt(&spd, 20, 80, 5);
        if (edit_param == 1) EditInt((uint8_t*)&off, -100, 100, 1);
        if (edit_param == 2) EditFloat(&gain, 0.5, 2, 0.1);
        if (edit_param == 3) EditFloat(&flt, 0.1, 0.9, 0.05);
        if (edit_param == 4) EditFloat(&MADGWICK_BETA, 0.01, 0.2, 0.01);
    }
}

// 3.1 静止
static void M1_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "1");
}

// 3.2 绕圈
static void M2_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "2");
    
    char line[17];
    sprintf(line, "lap:%d", lap);
    oled_show_string(1, 3, line);
    oled_show_string(1, 4, "start");
    oled_show_string(1, 5, "stop");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Mode2_Start();
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Mode2_Stop();
}

// 3.2.1 路线1
static void M2P1_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "path1");
    oled_show_string(1, 3, "start");
    oled_show_string(1, 4, "stop");
    oled_show_string(1, 5, "save");
    oled_show_string(1, 6, "delete");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Path_StartRecord(PATH_ID_MODE2_1);
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Path_StopRecord();
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) Path_SaveToFlash(PATH_ID_MODE2_1);
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) Path_Delete(PATH_ID_MODE2_1);
}

// 3.2.2 路线2
static void M2P2_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "path2");
    oled_show_string(1, 3, "start");
    oled_show_string(1, 4, "stop");
    oled_show_string(1, 5, "save");
    oled_show_string(1, 6, "delete");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Path_StartRecord(PATH_ID_MODE2_2);
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Path_StopRecord();
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) Path_SaveToFlash(PATH_ID_MODE2_2);
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) Path_Delete(PATH_ID_MODE2_2);
}

// 3.3 八字
static void M3_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "3");
    
    char line[17];
    sprintf(line, "cyc:%d", cyc);
    oled_show_string(1, 3, line);
    oled_show_string(1, 4, "start");
    oled_show_string(1, 5, "stop");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Mode3_Start();
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Mode3_Stop();
}

// 3.3.1 八字路线1
static void M3P1_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "path1");
    oled_show_string(1, 3, "start");
    oled_show_string(1, 4, "stop");
    oled_show_string(1, 5, "save");
    oled_show_string(1, 6, "delete");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Path_StartRecord(PATH_ID_MODE3_1);
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Path_StopRecord();
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) Path_SaveToFlash(PATH_ID_MODE3_1);
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) Path_Delete(PATH_ID_MODE3_1);
}

// 3.3.2 八字路线2
static void M3P2_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "path2");
    oled_show_string(1, 3, "start");
    oled_show_string(1, 4, "stop");
    oled_show_string(1, 5, "save");
    oled_show_string(1, 6, "delete");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Path_StartRecord(PATH_ID_MODE3_2);
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Path_StopRecord();
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) Path_SaveToFlash(PATH_ID_MODE3_2);
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) Path_Delete(PATH_ID_MODE3_2);
}

// 3.4 学习
static void M4_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "4");
    
    char line[17];
    sprintf(line, "pt:%d", pt);
    oled_show_string(1, 3, line);
    oled_show_string(1, 4, "path");
    oled_show_string(1, 5, "start");
    oled_show_string(1, 6, "stop");
}

// 3.4.1 学习路线
static void M4P_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "path");
    oled_show_string(1, 3, "start");
    oled_show_string(1, 4, "stop");
    oled_show_string(1, 5, "save");
    oled_show_string(1, 6, "delete");
    
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) Path_StartRecord(PATH_ID_MODE4);
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS) Path_StopRecord();
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS) Path_SaveToFlash(PATH_ID_MODE4);
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS) Path_Delete(PATH_ID_MODE4);
}

// 3.5 遥控
static void M5_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "5");
}
