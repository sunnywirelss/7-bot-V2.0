/**
  *************************************************************************************************
  * @file           Arm_four_move_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于演示舵机012的简单组合运动
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_four_move_test");
  MyArm.position_init();
  delay(3000);
}

void loop() {
    //姿态1
    MyArm.move_to_position(100.0, 200,232,500);
    MyArm.turn_steer_345_to_positon((word)2300,2600,2700,500);
    delay(510);
    //姿态2
    MyArm.move_to_position(-100.0, 200,232,700);
    MyArm.turn_steer_345_to_positon((word)1700,1400,1300,700);
    delay(710);
    //姿态3
    MyArm.move_to_position(-100.0, 200, 133,500);
    MyArm.turn_steer_345_to_positon((word)2500,2400,2700,500);
    delay(510);
    //姿态4
    MyArm.move_to_position(100.0, 200, 133,700);
    MyArm.turn_steer_345_to_positon((word)2047,2047,2047,700);    
    delay(710);
}



