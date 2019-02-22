/**
  *************************************************************************************************
  * @file           Arm_quality_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于机械臂出厂质量检测
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_quality_test");
  MyArm.position_init();
  delay(3000);
}

void loop() {

    //舵机012联动自检
    MyArm.move_to_position(100.0, 220,232,500);
    delay(510);
    MyArm.move_to_position(-100.0, 220,232,700);
    delay(710);
    MyArm.move_to_position(-100.0, 220, 133,500);
    delay(510);
    MyArm.move_to_position(100.0, 220, 133,700);
    delay(710);
    MyArm.move_to_position(100.0, 220,232,500);
    delay(510);
    MyArm.move_to_position((word)2047, 2047, 2047,400);
    delay(410);
   
    //舵机5自检
    MyArm.turn_steer_345_to_positon((word)2047,2047,1024,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)2047,2047,3047,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)2047,2047,2047,1000);
    delay(1000);
    //舵机4自检
    MyArm.turn_steer_345_to_positon((word)2047,1024,2047,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)2047,3047,2047,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)2047,2047,2047,1000);
    delay(1000);
     //舵机3自检
    MyArm.turn_steer_345_to_positon((word)1024,2047,2047,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)3047,2047,2047,2000);
    delay(2000);
    MyArm.turn_steer_345_to_positon((word)2047,2047,2047,1000);
    delay(1000);
}



