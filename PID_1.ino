#include "all_data_1.h"
//->
#include "GPS_path.h"
#include "structs.h"
//<-

String ans;
const char sep =',';

TaskHandle_t Task1;
double pgo_l, pgo_r;

void setup() {
  Serial.begin(115200);
  init_piezo();
  if (init_bmp()){delay_piezo(100);}
  delay(100);
  if (init_mpu()){delay_piezo(100);}
  delay(650);
//  init_gps();
  if (init_sd()){delay_piezo(100);}
  init_control();
  delay_piezo(1000);
  xTaskCreatePinnedToCore(control,"Task1",10000,NULL,1,&Task1,0);
  //-> 
  Log logs{};
  GPS_Path GPS{"", "GPS_text.txt", PathType::curve, 5, 20, 5, 20};
  Vec3D prevPos {lat(), lng(), alt()};
  //<-
}

void control( void * pvParameters ){
  while(true){
    
    read_control();
    
    if(!hand_control_flag){
      write_control_hand_petr();
      Serial.print(pitch);Serial.print(" ");
      Serial.println(roll);
    }
    else {
    // autopilot control //
    roll_ctrl_effect = servo_control[0] - roll_pid.ctrl((servo_control[0] - 1500) * (45/500), roll);
    pitch_ctrl_effect = servo_control[1] - pitch_pid.ctrl((servo_control[1] - 1500) * (45/500), pitch);
    /*
    double pgo_l = ((pitch_ctrl_effect > 1854)? 1854 : (pitch_ctrl_effect < 1146)? 1146 : pitch_ctrl_effect);
    
    double pgo_r = ((pitch_ctrl_effect > 1704)? 1704 : (pitch_ctrl_effect < 989)? 989 : pitch_ctrl_effect);
    double aileron_l = ((roll_ctrl_effect > 2000)? 2000 : (roll_ctrl_effect < 1000)? 1000 : roll_ctrl_effect);
    */
    
    //->
     Vec3D currPos {lat(), lng(), alt()};
     if (GPS_path.check_if_pos_in_point(currPos))
	     Vec3D targetPos = GPS_path.get_next_point(currPos);
     else
     	     Vec3D targetPos = GPS_path.get_target_point();
     Vec3D V = currPos - prevPos;
     Vec3D way = targetPos - currPos;
     Vec2D Vp, Wp;
     Vp = V.get_2d();
     Wp  = way.get_2d();
     double phi = angle(Vp,Wp) * side(Vp,Wp);
     double target_roll = Agl.ctrl(0,phi);
     double signal_h = Height.ctrl(targetPos,alt());
     prevPos = currPos;
    //<-
    
    //if (pitch_ctrl_effect == pitch_ctrl_effect)
    {
      pgo_l = ((pitch_ctrl_effect > 2000)? 2000 : (pitch_ctrl_effect < 1000)? 1000 : pitch_ctrl_effect);
      pgo_r = 3000-((pitch_ctrl_effect > 2000)? 2000 : (pitch_ctrl_effect < 1000)? 1000 : pitch_ctrl_effect);
    }
    Serial.print(pitch);Serial.print(" ");
    Serial.print(roll);Serial.print(" ");
    Serial.print(pgo_l);Serial.print(" ");
    Serial.println(pgo_r);
    
    servo_1.write(pgo_l);
    servo_2.write(pgo_r);
    servo_3.write(servo_control[2]);
    servo_4.write(servo_control[3]);
    servo_5.write(roll_ctrl_effect);
    servo_6.write(roll_ctrl_effect);
    ///////////////////////
    
    }
    vTaskDelay(1);
  } 
}

void loop() {
//  update_gps();
  update_mpu();
  ans = (
    "\n" +
    String(servo_control[0]) + sep +
    String(servo_control[1]) + sep +
    String(servo_control[2]) + sep +
    String(servo_control[3]) + sep +
    String(servo_control[4]) + sep +
    String(servo_control[5]) + sep +
    String(servo_control[6]) + sep +
    String(servo_control[7]) + sep +
    String(servo_control[8]) + sep +
    String(servo_control[9]) + sep +
    String(roll,2) + sep +
    String(pitch,2) + sep +
    String(yaw,2) + sep +
    String(alt()) + sep +
//    String(lat(),6) + sep + String(lng(),6) + sep +
    String(micros()));
    
  to_file(ans);   
}
