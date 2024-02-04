#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

class mqtt_sub_client:
    def __init__(self):
        self.ts = TouchSensor()
        self.leds = Leds()
        self.m = LargeMotor(OUTPUT_A)
        # Set Parmater
        self.broker_address = "localhost"
        self.port = 1883
        self.topic = "ev3_motor"
        self.timeout = 60
        
        # MQTTの接続設定
        self.client = mqtt.Client()     
        self.client.on_connect = self.on_connect # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect # 切断時のコールバックを登録
        self.client.on_message = self.on_message
        
        self.client.connect(self.broker_address, self.port, self.timeout)
        self.client.loop_forever() # 永久ループして待ち続ける
        
    def on_connect(self, client, userdata, flag, rc):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        self.client.subscribe(self.topic)  # subするトピックを設定 
        
    def on_disconnect(self, client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")
    
    def on_message(self, client, userdata, msg):
        received_data = msg.payload.decode("utf-8").split(',')
        # msg.topicにトピック名が，msg.payloadに届いたデータ本体が入っている
        print("Received message '" + str(msg.payload) + "' on topic '" + msg.topic + "' with QoS " + str(msg.qos))
        print("Received messagedata:", received_data)
        self.motorrun(int(received_data[0]), float(received_data[1]), int(received_data[2]), float(received_data[3]))
        time.sleep(0.1)
    
    def motorrun(self, motor_a_speed : int, motor_a_rotation : float, motor_b_speed : int, motor_b_rotation : float):
        if(motor_a_rotation == 0.0 ):
            self.m.on(SpeedPercent(motor_a_speed), brake = True)
        elif (motor_a_speed == 0 ):
            self.m.off()
        else:
            self.m.on_for_rotations(SpeedPercent(motor_a_speed), motor_a_rotation, brake = True)
        
        if(motor_b_rotation == 0.0 ):
            self.m.on(SpeedPercent(motor_b_speed), brake = True)
        elif (motor_b_speed == 0 ):
            self.m.off()
        else:
            self.m.on_for_rotations(SpeedPercent(motor_b_speed), motor_b_rotation, brake = True)
        
        
            

if __name__ == "__main__":
    client = mqtt_sub_client()
