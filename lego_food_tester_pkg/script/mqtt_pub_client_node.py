#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
import rospy
from lego_food_tester_pkg.msg import motor

class mqtt_pub_client:
    def __init__(self):
        # Set Parmater
        self.broker_address = "ev3dev.local"
        # self.broker_address = "localhost"
        self.port = 1883
        self.topic = "ev3_motor"
        self.timeout = 60
        
        # MQTTの接続設定
        self.client = mqtt.Client()                      # クラスのインスタンス(実体)の作成       
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        
        self.client.connect(self.broker_address, self.port, self.timeout)
        self.client.loop_start()
        
        self.publish(self.topic, 0, 0, 0, 0)
            
    def on_connect(self, client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
    
    def on_disconnect(self, client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")
    
    def on_publish(self, client, userdata, mid):
        print("publish: {0}".format(mid))
    
    def publish(self, topic : str, motor_a_speed : int, motor_a_rotation : float, motor_b_speed : int, motor_b_rotation : float):
        data_to_send = f"{str(motor_a_speed)},{'{:.2f}'.format(motor_a_rotation)},{str(motor_b_speed)},{'{:.2f}'.format(motor_b_rotation)}" # カンマで区切ったデータ
        self.client.publish(topic, data_to_send)    # トピック名とメッセージを決めて送信
        time.sleep(0.1)

class listener:
    def __init__(self):
        rospy.init_node('mqtt_pub_client_node')
        rospy.Subscriber("ev3_motor", motor, self.callback)
        self.mqtt_client = mqtt_pub_client()
        rospy.spin()
    
    def callback(self, msg):
        rospy.loginfo("publish")
        self.mqtt_client.publish(msg.topic_name, int(msg.motor_a_speed), float(msg.motor_a_rotation), int(msg.motor_b_speed), float(msg.motor_b_rotation))



if __name__ == "__main__":
    listener()