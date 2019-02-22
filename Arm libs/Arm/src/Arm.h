#ifndef ARM_H
#define ARM_H

//#define START_PROTECT

#include <Arduino.h>
#include <Serial_arm.h>
#include <HardwareSerial.h>
#include <Steer.h>
#include <EEPROM.h>
#include <PVector.h>
#include <TimerOne.h>
#include "Arm_protect.h"

#define USB_SER                     &Serial                                            //USB串口设置

//mechanical constraints        机械臂尺寸，详细尺寸说明见说明书
const double a = 120, b = 40, c = 188, d = 24.28, e = 79, f = 21.6, g = 12, h = 34.8;

/*************函数详细说明请参照CPP文件*************/

class Arm{

private:
    HardwareSerial *comSer;
    
    int *pos_goal;                                                                             //存储目标位置
    
    byte Steer_Detect();
    void Para_Init();
    
    short Rad2Pos(double rad);
    short Angle2Pos(double angle);
    double Pos2Angle(double pos);
    double Angle2Rad(double angle);
  
public:
    //word clc_Times_Total;                                                              //舵机从当前位置到达目标位置的总耗时

    byte Steer_Num;
    short *offPos;
    double *theta;

    Steer *steer[7] = {NULL};
    
    void begin(HardwareSerial *desireSer);
    
    void position_init(void);
    
    void inverse_movement(double x_ , double y_, double z_);
    void inverse_movement(PVector pt);
    
    void move_to_position( double x_ , double y_, double z_ , word runtime);
    void move_to_position( PVector pt , word runtime);
    void move_to_position( word pos0 , word pos1, word pos2 , word runtime);

    void offset_by_pos(byte id, short offset);
    void offset_by_angle(byte id, double angle);
    
    void Set_Arm_Torque_On(void);
    void Set_Arm_Torque_Off(void);
    
    boolean turn_steer_345_to_positon(word pos3 , word pos4, word pos5 , word runtime);
    boolean turn_steer_345_to_positon(double angle3 , double angle4, double angle5 , word runtime);
    
    void Get_Offset();
    double Rad2Angle(double rad);
    double Pos2Rad(word pos);
};

extern Arm MyArm;

double mapFloat(double val, double in_min, double in_max, double out_min, double out_max);

#endif
