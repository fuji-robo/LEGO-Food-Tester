#define DEBUG_ROS true
#define DEBUG_SERIAL false

#include <HX711.h>
#include <ros.h>
#include <std_msgs/Float32.h> // ROSのstd_msgs/Int64メッセージを使用

#if DEBUG_ROS
ros::NodeHandle nh;

std_msgs::Float32 float_msg; // Int64型のメッセージを作成

ros::Publisher pub("loadcell", &float_msg); // パブリッシャーを定義（"topic_name" は使用するトピック名）
#endif

const int LOADCELL_DOUT_PIN = 8;
const int LOADCELL_SCK_PIN = 9;

HX711 scale;

void setup() {
  #if DEBUG_SERIAL
  Serial.begin(57600);
  #endif

  #if DEBUG_ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub); // パブリッシャーを登録
  #endif
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(); //583.1265　513.9335
  scale.tare();

  scale.set_scale(425);
}

void loop() {
  long val=0;

  if (scale.is_ready()) {
    float val = scale.get_units(10);
    #if DEBUG_ROS
    float_msg.data = val; // メッセージにデータをセット
    pub.publish(&float_msg); // メッセージをパブリッシュ
    #endif
    #if DEBUG_SERIAL
    Serial.println(val);
    delay(100);
    #endif
  }
  else {
    #if DEBUG_SERIAL
    Serial.println("HX711 not found.");
    #endif
    #if DEBUG_ROS
    nh.logwarn("Warnings: HX711 not found."); // ROSノードを実行
    #endif
  }
  #if DEBUG_ROS
  nh.spinOnce(); // ROSノードを実行
  delay(100);
  #endif

}
