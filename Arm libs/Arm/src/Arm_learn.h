#ifndef ARM_LEARN_H
#define ARM_LEARN_H

#include <Arduino.h>
#include <Serial_arm.h>
#include <HardwareSerial.h>
#include <Steer.h>
#include <EEPROM.h>
#include <PVector.h>
#include <Arm.h>

#define BTN0_PIN                   2                                                //����0��ѧϰ��ʼ���������                                               
#define BTN1_PIN                   3                                                //����1��λ���¼��λ�����ְ�ť
#define BUZZER_PIN               37                                              //����������

#define RUN_VELOCITY           0.3                                             //�����ٶ�
#define RUN_TIME                   2000                                         //����ʱ���趨���޸ĸ�ֵ���Ըı��е�۵��ٶ�


/*****class����ĺ�������˵����ο���Ӧ��Դ�ļ�****/

class Arm_learn{

private:
        
public:
    void Arm_learn_(void);
    
    int button0_time_cnt;                              //����0�İ���ʱ������
    int button1_time_cnt;                              //����1�İ���ʱ����������������ִ������˶��ٸ�10���룩
    int rom_offset;                                         //��¼EEPROM�ڵĵ�ַƫ��
    word record_point_num;                          //��¼λ�����Ŀ
    
    boolean start_learn_flag;                          //��ʼѧϰ��־λ
    boolean record_flag;                                //λ���¼��־λ
    
    void button_learn_detect(void);
    void record_one_point(void);
    void start_learn(void);
    void reappear_learn(void);
    word get_steer_positon(byte i);
    
    word get_point_dis();    
    void set_reappear_velocity();
};

extern Arm_learn MyArm_learn;

#endif
