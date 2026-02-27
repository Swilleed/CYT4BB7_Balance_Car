#include "zf_common_headfile.h"
#include "menu.h"
#include "app.h"
#include "balance_task.h"
#include "pid_controller.h"
#include "flash_logger.h"

// ==================== 从其他模块extern的变量 ====================
extern float forward_speed;
extern uint16_t task2_lap_count;
extern uint16_t task3_lap_count;
extern AppTask_enum current_task;
extern pid_controller_t yaw_pid;
extern pid_controller_t line_pid;
extern PID_TypeDef _Angle;
extern PID_TypeDef _Angle_Speed;
extern PID_TypeDef _Speed;
extern PID_TypeDef _Dir;
extern float BaseAngle;
extern float Roll;
extern float X_gyro;
extern uint16_t path_node_count;

// ==================== 全局菜单变量 ====================
Menu *CurrentMenu;
uint8_t CurrentSelection = 0;
static uint8_t edit_mode = 0;
static uint8_t edit_param = 0;
static uint8_t need_save = 0;

// ==================== 菜单对象声明 ====================
static Menu MainMenu;
static Menu PIDMenu;
static Menu MoveMenu;
static Menu ModeMenu;

static Menu BalanceAngleMenu;
static Menu BalanceGyroMenu;
static Menu BalanceSpeedMenu;
static Menu BalanceDirMenu;
static Menu YawPIDMenu;
static Menu LinePIDMenu;

static Menu Mode1Menu;
static Menu Mode2Menu;
static Menu Mode3Menu;
static Menu Mode4Menu;
static Menu Mode5Menu;
static Menu Mode4PathMenu;

// ==================== Flash参数保存 ====================
#define PARAM_FLASH_PAGE 500

typedef struct {
    float angle_p, angle_i, angle_d;
    float gyro_p, gyro_i, gyro_d;
    float speed_p, speed_i, speed_d;
    float dir_p, dir_i, dir_d;
    float yaw_p, yaw_i, yaw_d;
    float line_p, line_i, line_d;
    float base_angle;
    float forward_spd;
    uint16_t magic;
} ParamSave_t;

static void SaveParams(void)
{
    ParamSave_t param;
    param.angle_p = _Angle.Kp;
    param.angle_i = _Angle.Ki;
    param.angle_d = _Angle.Kd;
    param.gyro_p = _Angle_Speed.Kp;
    param.gyro_i = _Angle_Speed.Ki;
    param.gyro_d = _Angle_Speed.Kd;
    param.speed_p = _Speed.Kp;
    param.speed_i = _Speed.Ki;
    param.speed_d = _Speed.Kd;
    param.dir_p = _Dir.Kp;
    param.dir_i = _Dir.Ki;
    param.dir_d = _Dir.Kd;
    param.yaw_p = yaw_pid.Kp;
    param.yaw_i = yaw_pid.Ki;
    param.yaw_d = yaw_pid.Kd;
    param.line_p = line_pid.Kp;
    param.line_i = line_pid.Ki;
    param.line_d = line_pid.Kd;
    param.base_angle = BaseAngle;
    param.forward_spd = forward_speed;
    param.magic = 0x5A5A;
    Logger_WriteBlock(PARAM_FLASH_PAGE, &param, sizeof(ParamSave_t));
}

static void LoadParams(void)
{
    ParamSave_t param;
    Logger_ReadBlock(PARAM_FLASH_PAGE, &param, sizeof(ParamSave_t));
    if (param.magic == 0x5A5A) {
        _Angle.Kp = param.angle_p;
        _Angle.Ki = param.angle_i;
        _Angle.Kd = param.angle_d;
        _Angle_Speed.Kp = param.gyro_p;
        _Angle_Speed.Ki = param.gyro_i;
        _Angle_Speed.Kd = param.gyro_d;
        _Speed.Kp = param.speed_p;
        _Speed.Ki = param.speed_i;
        _Speed.Kd = param.speed_d;
        _Dir.Kp = param.dir_p;
        _Dir.Ki = param.dir_i;
        _Dir.Kd = param.dir_d;
        yaw_pid.Kp = param.yaw_p;
        yaw_pid.Ki = param.yaw_i;
        yaw_pid.Kd = param.yaw_d;
        line_pid.Kp = param.line_p;
        line_pid.Ki = param.line_i;
        line_pid.Kd = param.line_d;
        BaseAngle = param.base_angle;
        forward_speed = param.forward_spd;
    }
}

// ==================== 编辑函数 ====================
static void EditFloat(float *val, float min, float max, float step)
{
    if (key_get_state(KEY_UP) == KEY_SHORT_PRESS) {
        *val += step;
        if (*val > max) *val = max;
        need_save = 1;
    }
    if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS) {
        *val -= step;
        if (*val < min) *val = min;
        need_save = 1;
    }
}

static void EditInt(uint8_t *val, uint8_t min, uint8_t max, uint8_t step)
{
    if (key_get_state(KEY_UP) == KEY_SHORT_PRESS) {
        *val += step;
        if (*val > max) *val = max;
        need_save = 1;
    }
    if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS) {
        *val -= step;
        if (*val < min) *val = min;
        need_save = 1;
    }
}

// ==================== 菜单导航函数 ====================
static void NavigateToChild(void)
{
    if (CurrentSelection < CurrentMenu->child_count)
    {
        CurrentMenu = CurrentMenu->children[CurrentSelection];
        CurrentSelection = 0;
        edit_mode = 0;
        edit_param = 0;
    }
}

static void NavigateToParent(void)
{
    if (CurrentMenu->parent != NULL)
    {
        if (need_save) {
            SaveParams();
            need_save = 0;
        }
        CurrentMenu = CurrentMenu->parent;
        CurrentSelection = 0;
        edit_mode = 0;
        edit_param = 0;
    }
}

static void HandleInput(void)
{
    if (CurrentMenu->child_count > 0)
    {
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS)
        {
            CurrentSelection = (CurrentSelection + 1) % CurrentMenu->child_count;
        }
        else if (key_get_state(KEY_UP) == KEY_SHORT_PRESS)
        {
            CurrentSelection = (CurrentSelection + CurrentMenu->child_count - 1) % CurrentMenu->child_count;
        }
        else if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS)
        {
            NavigateToChild();
        }
        else if (CurrentMenu->parent != NULL && key_get_state(KEY_BACK) == KEY_SHORT_PRESS)
        {
            NavigateToParent();
        }
    }
    else
    {
        if (key_get_state(KEY_BACK) == KEY_SHORT_PRESS)
        {
            if (edit_mode && need_save) {
                SaveParams();
                need_save = 0;
            }
            edit_mode = 0;
            edit_param = 0;
            NavigateToParent();
        }
    }
}

void DisplayMenu(void)
{
    HandleInput();
    if (CurrentMenu->child_count == 0)
    {
        if (CurrentMenu->action != NULL)
        {
            CurrentMenu->action();
        }
        return;
    }
    oled_clear();
    oled_show_string(1, 1, CurrentMenu->title);
    for (uint8_t i = 0; i < CurrentMenu->child_count; i++)
    {
        uint8_t row = i + 2;
        char line[17];
        snprintf(line, sizeof(line), "%u %c%s", i + 1, (i == CurrentSelection) ? '>' : ' ', CurrentMenu->children[i]->title);
        oled_show_string(1, row, line);
    }
}

void Menu_AddChild(Menu *parent, Menu *child)
{
    if (parent->child_count < MENU_MAX_CHILDREN)
    {
        parent->children[parent->child_count] = child;
        parent->child_count++;
        child->parent = parent;
    }
}

// ==================== Action函数 ====================

static void BalanceAngle_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Angle PID");
    char line[17];
    sprintf(line, "P:%.2f", _Angle.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", _Angle.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", _Angle.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&_Angle.Kp, 0, 10, 0.1);
        if (edit_param == 1) EditFloat(&_Angle.Ki, 0, 2, 0.01);
        if (edit_param == 2) EditFloat(&_Angle.Kd, 0, 5, 0.1);
    }
}

static void BalanceGyro_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Gyro PID");
    char line[17];
    sprintf(line, "P:%.2f", _Angle_Speed.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", _Angle_Speed.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", _Angle_Speed.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&_Angle_Speed.Kp, 0, 5, 0.1);
        if (edit_param == 1) EditFloat(&_Angle_Speed.Ki, 0, 1, 0.01);
        if (edit_param == 2) EditFloat(&_Angle_Speed.Kd, 0, 5, 0.1);
    }
}

static void BalanceSpeed_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Speed PID");
    char line[17];
    sprintf(line, "P:%.2f", _Speed.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", _Speed.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", _Speed.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&_Speed.Kp, 0, 5, 0.1);
        if (edit_param == 1) EditFloat(&_Speed.Ki, -1, 1, 0.01);
        if (edit_param == 2) EditFloat(&_Speed.Kd, 0, 5, 0.1);
    }
}

static void BalanceDir_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Dir PID");
    char line[17];
    sprintf(line, "P:%.2f", _Dir.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", _Dir.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", _Dir.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&_Dir.Kp, 0, 1, 0.01);
        if (edit_param == 1) EditFloat(&_Dir.Ki, 0, 0.5, 0.01);
        if (edit_param == 2) EditFloat(&_Dir.Kd, 0, 10, 0.1);
    }
}

static void YawPID_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Yaw PID");
    char line[17];
    sprintf(line, "P:%.2f", yaw_pid.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", yaw_pid.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", yaw_pid.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&yaw_pid.Kp, 0, 5, 0.1);
        if (edit_param == 1) EditFloat(&yaw_pid.Ki, 0, 1, 0.01);
        if (edit_param == 2) EditFloat(&yaw_pid.Kd, 0, 2, 0.05);
    }
}

static void LinePID_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Line PID");
    char line[17];
    sprintf(line, "P:%.2f", line_pid.Kp);
    oled_show_string(1, 3, line);
    sprintf(line, "I:%.2f", line_pid.Ki);
    oled_show_string(1, 4, line);
    sprintf(line, "D:%.2f", line_pid.Kd);
    oled_show_string(1, 5, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 2) edit_param++;
        if (edit_param == 0) EditFloat(&line_pid.Kp, 0, 5, 0.1);
        if (edit_param == 1) EditFloat(&line_pid.Ki, 0, 1, 0.01);
        if (edit_param == 2) EditFloat(&line_pid.Kd, 0, 2, 0.05);
    }
}

static void Move_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Move Params");
    char line[17];
    sprintf(line, "BaseAngle:%.2f", BaseAngle);
    oled_show_string(1, 3, line);
    sprintf(line, "Speed:%d", (int)forward_speed);
    oled_show_string(1, 4, line);
    
    if (key_get_state(KEY_SELECT) == KEY_SHORT_PRESS) {
        edit_mode = !edit_mode;
        edit_param = 0;
        need_save = 0;
    }
    
    if (edit_mode) {
        oled_show_string(1, 7, "EDIT");
        if (key_get_state(KEY_UP) == KEY_SHORT_PRESS && edit_param > 0) edit_param--;
        if (key_get_state(KEY_DOWN) == KEY_SHORT_PRESS && edit_param < 1) edit_param++;
        if (edit_param == 0) EditFloat(&BaseAngle, 0, 5, 0.05);
        if (edit_param == 1) EditInt((uint8_t*)&forward_speed, 5, 50, 1);
    }
}

static void Mode1_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Stand Balance");
    char line[17];
    if(current_task == TASK_IDLE || current_task == TASK1_RUNNING) {
        oled_show_string(1, 3, "RUNNING");
        sprintf(line, "Angle:%.1f", Roll);
        oled_show_string(1, 4, line);
        sprintf(line, "Gyro:%.1f", X_gyro);
        oled_show_string(1, 5, line);
    } else {
        oled_show_string(1, 3, "Other Task");
    }
    oled_show_string(1, 7, "IDLE=Stand");
}

static void Mode2_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Circle");
    char line[17];
    if(current_task == TASK2_RUNNING) {
        oled_show_string(1, 3, "RUNNING");
        sprintf(line, "Lap:%d/1", task2_lap_count);
        oled_show_string(1, 4, line);
    } else if(current_task == TASK_IDLE) {
        oled_show_string(1, 3, "Ready");
        sprintf(line, "Last:%d", task2_lap_count);
        oled_show_string(1, 4, line);
    } else {
        oled_show_string(1, 3, "Busy");
    }
    oled_show_string(1, 5, "KEY1:Start");
    oled_show_string(1, 6, "KEY2:Stop");
    
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS && current_task == TASK_IDLE) {
        App_Start_Task2();
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        App_Stop_CurrentTask();
    }
}

static void Mode3_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "8-shape");
    char line[17];
    if(current_task == TASK3_RUNNING) {
        oled_show_string(1, 3, "RUNNING");
        sprintf(line, "Lap:%d/4", task3_lap_count);
        oled_show_string(1, 4, line);
    } else if(current_task == TASK_IDLE) {
        oled_show_string(1, 3, "Ready");
        sprintf(line, "Last:%d", task3_lap_count);
        oled_show_string(1, 4, line);
    } else {
        oled_show_string(1, 3, "Busy");
    }
    oled_show_string(1, 5, "KEY1:Start");
    oled_show_string(1, 6, "KEY2:Stop");
    
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS && current_task == TASK_IDLE) {
        App_Start_Task3();
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        App_Stop_CurrentTask();
    }
}

static void Mode4_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Learn");
    char line[17];
    if(current_task == TASK4_RECORDING) {
        oled_show_string(1, 3, "RECORDING");
        sprintf(line, "Pts:%d", path_node_count);
        oled_show_string(1, 4, line);
    } else if(current_task == TASK4_PLAYBACK) {
        oled_show_string(1, 3, "PLAYBACK");
        sprintf(line, "Pts:%d", path_node_count);
        oled_show_string(1, 4, line);
    } else if(current_task == TASK_IDLE) {
        oled_show_string(1, 3, "Ready");
        sprintf(line, "Saved:%d", path_node_count);
        oled_show_string(1, 4, line);
    } else {
        oled_show_string(1, 3, "Busy");
    }
    oled_show_string(1, 6, "Path >");
}

static void Mode5_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Remote");
    if(current_task == TASK5_RUNNING) {
        oled_show_string(1, 3, "RUNNING");
    } else if(current_task == TASK_IDLE) {
        oled_show_string(1, 3, "Ready");
    } else {
        oled_show_string(1, 3, "Busy");
    }
    oled_show_string(1, 5, "KEY1:Start");
    oled_show_string(1, 6, "KEY2:Stop");
    
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS && current_task == TASK_IDLE) {
        App_Start_Task5();
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        App_Stop_CurrentTask();
    }
}

static void Mode4Path_Action(void)
{
    oled_clear();
    oled_show_string(1, 1, "Path Ops");
    if(current_task == TASK4_RECORDING) {
        oled_show_string(1, 2, "RECORDING");
    } else if(current_task == TASK4_PLAYBACK) {
        oled_show_string(1, 2, "PLAYBACK");
    }
    oled_show_string(1, 3, "1:Record");
    oled_show_string(1, 4, "2:Stop");
    oled_show_string(1, 5, "3:Play");
    oled_show_string(1, 6, "4:Reset");
    
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        App_Start_Task4_Record();
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        App_Stop_CurrentTask();
    }
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS) {
        App_Start_Task4_Playback();
    }
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS) {
        App_Stop_CurrentTask();
        Odometry_Reset();
    }
}

Menu *InitMenu(void)
{
    LoadParams();
    
    // 设置菜单标题
    MainMenu.title = "Main Menu";
    PIDMenu.title = "PID";
    MoveMenu.title = "MOVE";
    ModeMenu.title = "MODE";
    
    BalanceAngleMenu.title = "Angle PID";
    BalanceGyroMenu.title = "Gyro PID";
    BalanceSpeedMenu.title = "Speed PID";
    BalanceDirMenu.title = "Dir PID";
    YawPIDMenu.title = "Yaw PID";
    LinePIDMenu.title = "Line PID";
    
    Mode1Menu.title = "1 Stand";
    Mode2Menu.title = "2 Circle";
    Mode3Menu.title = "3 8-shape";
    Mode4Menu.title = "4 Learn";
    Mode5Menu.title = "5 Remote";
    Mode4PathMenu.title = "Path";
    
    // 设置Action函数
    BalanceAngleMenu.action = BalanceAngle_Action;
    BalanceGyroMenu.action = BalanceGyro_Action;
    BalanceSpeedMenu.action = BalanceSpeed_Action;
    BalanceDirMenu.action = BalanceDir_Action;
    YawPIDMenu.action = YawPID_Action;
    LinePIDMenu.action = LinePID_Action;
    MoveMenu.action = Move_Action;
    Mode1Menu.action = Mode1_Action;
    Mode2Menu.action = Mode2_Action;
    Mode3Menu.action = Mode3_Action;
    Mode4Menu.action = Mode4_Action;
    Mode5Menu.action = Mode5_Action;
    Mode4PathMenu.action = Mode4Path_Action;
    
    // 构建菜单树
    Menu_AddChild(&MainMenu, &PIDMenu);
    Menu_AddChild(&MainMenu, &MoveMenu);
    Menu_AddChild(&MainMenu, &ModeMenu);
    
    Menu_AddChild(&PIDMenu, &BalanceAngleMenu);
    Menu_AddChild(&PIDMenu, &BalanceGyroMenu);
    Menu_AddChild(&PIDMenu, &BalanceSpeedMenu);
    Menu_AddChild(&PIDMenu, &BalanceDirMenu);
    Menu_AddChild(&PIDMenu, &YawPIDMenu);
    Menu_AddChild(&PIDMenu, &LinePIDMenu);
    
    Menu_AddChild(&ModeMenu, &Mode1Menu);
    Menu_AddChild(&ModeMenu, &Mode2Menu);
    Menu_AddChild(&ModeMenu, &Mode3Menu);
    Menu_AddChild(&ModeMenu, &Mode4Menu);
    Menu_AddChild(&ModeMenu, &Mode5Menu);
    
    Menu_AddChild(&Mode4Menu, &Mode4PathMenu);
    
    CurrentMenu = &MainMenu;
    CurrentSelection = 0;
    
    return CurrentMenu;
}
