#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from tkinter import *
from tkinter import ttk
from tkinter.font import Font
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from lego_food_tester_pkg.msg import motor
from std_msgs.msg import Float32

class MainGUI:
    def __init__(self, root):
        self.control = motor()
        self.control.topic_name = "ev3_motor"
        self.control.motor_a_speed = 0
        self.control.motor_a_rotation = 0.0
        self.control.motor_b_speed = 0
        self.control.motor_b_rotation = 0.0
        
        # flag
        self.bt_rack_for_ctrl_flag = 0
        self.bt_cylinder_for_ctrl_flag = 0
        
        # graph
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        self.line, = self.ax.plot([], [])
        self.ax.set_ylim(0, 5000)
        self.ax.set_xlabel('Time [s]')
        self.ax.set_ylabel('Weight[g]')
        self.times = []
        self.values = []
        self.start_time = time.time()
        self.max_display_time = 10 # 表示する最大の時間（秒）
        self.time_interval = 0.1   # 時間の刻み幅（秒）
        self.gram_data = 0.0
        self.is_update_flag = True
        
        
        
        
        
        # Tkinterの文字サイズ定義
        self.pt12_font = Font(size=12)
        self.pt10_font = Font(size=10)
        self.pt50_font = Font(size=50)
        
        # ROSノードの初期設定
        rospy.init_node('gui_node')
        
        # ROS Publisherの設置
        self.control_pub = rospy.Publisher("ev3_motor", motor, queue_size=10)
        self.loadcell_sub = rospy.Subscriber('loadcell', Float32, self.loadcell_callback)
        
        # Tkinterの初期設定
        self.root = root
        self.root.title("LEGO FOOD TESTER")
        self.root.geometry('1920x1080')
        self.repeat_action()
        
        # LabelFrameを配置するためのフレームを作成
        self.frame = Frame(root, width=1920, height=1080)
        self.frame.propagate(False)
        self.frame.pack()
        
        # Frameの設定
        ## 1列目
        self.frame_control = LabelFrame(self.frame, text="Control",
                                        width=500, height=250, 
                                        cursor='hand2', labelanchor='n',
                                        borderwidth=1, font = self.pt12_font)
        self.frame_calibration = LabelFrame(self.frame, text="Calibration",
                                            width=500, height=250,
                                            cursor='hand2',labelanchor='n',
                                            borderwidth=1, font = self.pt12_font)
        self.frame_measurement = LabelFrame(self.frame, text="Measurement", 
                                            width=500, height=250, cursor='hand2',
                                            labelanchor='n', borderwidth=1,
                                            font = self.pt12_font)
        self.frame_Load = LabelFrame(self.frame, text="Load",
                                     width=500, height=250, 
                                     cursor='hand2', labelanchor='n',
                                     borderwidth=1, font = self.pt12_font)
        ## 2列目
        self.frame_graph = LabelFrame(self.frame,text="Graph", 
                                       width=1300, height=750, 
                                       cursor='hand2', labelanchor='n',
                                       borderwidth=1, font = self.pt12_font)
        self.frame_result = LabelFrame(self.frame,text="Result", 
                                       width=1300, height = 250, 
                                       cursor='hand2', labelanchor='n',
                                       borderwidth=1, font = self.pt12_font)
        
        # Frameサイズを固定
        self.frame_control.propagate(False)
        self.frame_calibration.propagate(False)
        self.frame_measurement.propagate(False)
        self.frame_Load.propagate(False)
        self.frame_graph.propagate(False)
        self.frame_result.propagate(False)
        
        # Frameサイズを固定
        ## 1列目
        self.frame_control.grid(row = 0, column = 0)
        self.frame_calibration.grid(row = 1, column = 0)
        self.frame_measurement.grid(row = 2, column = 0)
        self.frame_Load.grid(row = 3, column = 0)
        ## 2列目
        self.frame_graph.grid(row = 0, column = 1, rowspan = 3, padx=20)
        self.frame_result.grid(row = 3, column = 1, padx=20)
        
        self.cfg_frame_control()
        self.cfg_frame_calibration()
        self.cfg_frame_measurement()
        # self.cfg_frame_Load():
        self.cfg_frame_graph()
        self.cfg_frame_result()
    
    def cfg_frame_control(self):
        # 共通
        self.speed_values = list(map(str, range(101)))
        self.rotation_values = np.arange(0, 10.25, 0.25).tolist()
        
        # Label Frameの作成
        # 1列目
        self.frame_control_rack = LabelFrame(self.frame_control, text="Rack and Pinion",
                                        width=200, height=200, 
                                        cursor='hand2', labelanchor='n',
                                        borderwidth=1, font = self.pt10_font)
        # 2列目
        self.frame_control_cylinder = LabelFrame(self.frame_control, text="Cylinder",
                                        width=200, height=200, 
                                        cursor='hand2', labelanchor='n',
                                        borderwidth=1, font = self.pt10_font)
        
        # 配置の設定
        self.frame_control_rack.propagate(False)
        self.frame_control_cylinder.propagate(False)
        self.frame_control_rack.grid(row = 0, column = 0, padx=20, pady=25)
        self.frame_control_cylinder.grid(row = 0, column = 1, padx=20, pady=25)
        
        # 1列目(Rack and Pinion)
        ## Buttonの作成
        self.bt_rack_up_for_ctrl = Button(self.frame_control_rack, text='⬆UP', relief='raised', 
                                    bg='pink', width=10, cursor='hand2', font = self.pt10_font, 
                                    command=lambda:self.bt_rack_for_ctrl_callbark(toggle = 1),
                                    repeatdelay=100, repeatinterval=500)
        self.bt_rack_down_for_ctrl = Button(self.frame_control_rack, text='⬇DOWN', relief='raised', 
                                    bg='sky blue', width=10, cursor='hand2', font = self.pt10_font, 
                                    command=lambda:self.bt_rack_for_ctrl_callbark(toggle = 0),
                                    repeatdelay=100, repeatinterval=500)
        
        ## ラベルの作成
        self.rack_speed_label = Label(self.frame_control_rack, text = 'Speed', width=8)
        self.rack_rotation_label = Label(self.frame_control_rack,text = 'Rotation', width=8)
        
        ## Comboboxの作成
        self.rack_speed_combo_box = ttk.Combobox(self.frame_control_rack, values = self.speed_values, width=5)
        self.rack_rotation_combo_box = ttk.Combobox(self.frame_control_rack, values = self.rotation_values, width=5)
        self.rack_speed_combo_box.set(self.speed_values[50])
        self.rack_rotation_combo_box.set(self.rotation_values[0])
        
        # 2列目(Cylinder)
        ## Buttonの作成
        self.bt_cylinder_up_for_ctrl = Button(self.frame_control_cylinder, text='⬆UP', relief='raised', 
                                    bg='pink', width=10, cursor='hand2', font = self.pt10_font, 
                                    command=lambda:self.bt_cylinder_for_ctrl_callbark(toggle = 1),
                                    repeatdelay=100, repeatinterval=500)
        self.bt_cylinder_down_for_ctrl = Button(self.frame_control_cylinder, text='⬇DOWN', relief='raised', 
                                    bg='sky blue', width=10, cursor='hand2', font = self.pt10_font, 
                                    command=lambda:self.bt_cylinder_for_ctrl_callbark(toggle = 0),
                                    repeatdelay=100, repeatinterval=500)
        
        ## ラベルの作成
        self.cylinder_speed_label = Label(self.frame_control_cylinder ,text = 'Speed', width=8)
        self.cylinder_rotation_label = Label(self.frame_control_cylinder, text = 'Rotation', width=8)
        
        ## Comboboxの作成
        self.cylinder_speed_combo_box = ttk.Combobox(self.frame_control_cylinder, values = self.speed_values, width=5)
        self.cylinder_rotation_combo_box = ttk.Combobox(self.frame_control_cylinder, values = self.rotation_values, width=5)
        self.cylinder_speed_combo_box.set(self.speed_values[50])
        self.cylinder_rotation_combo_box.set(self.rotation_values[0])
        
        # 配置
        ## 1行目
        self.bt_rack_up_for_ctrl.grid(row = 0, column = 0, columnspan = 2, padx=20, pady = 5)
        self.bt_rack_down_for_ctrl.grid(row = 1, column = 0, columnspan = 2, padx=20, pady = 5)
        self.rack_speed_label.grid(row = 2, column = 0, padx=20, pady = 5)
        self.rack_rotation_label.grid(row = 3, column = 0, padx=20, pady = 5)
        self.rack_speed_combo_box.grid(row = 2, column = 1, padx=20, pady = 5)
        self.rack_rotation_combo_box.grid(row = 3, column = 1, padx=20, pady = 5)
        
        
        ## 2行目
        self.bt_cylinder_up_for_ctrl.grid(row=0, column=0, columnspan=2, padx=20, pady=5)
        self.bt_cylinder_down_for_ctrl.grid(row=1, column=0, columnspan=2, padx=20, pady=5)
        self.cylinder_speed_label.grid(row=2, column=0, padx=20, pady=5)
        self.cylinder_rotation_label.grid(row=3, column=0, padx=20, pady=5)
        self.cylinder_speed_combo_box.grid(row=2, column=1, padx=20, pady=5)
        self.cylinder_rotation_combo_box.grid(row=3, column=1, padx=20, pady=5)
        
        self.bt_rack_up_for_ctrl.bind("<ButtonRelease>", lambda event: self.bt_rack_for_ctrl_released())
        self.bt_rack_down_for_ctrl.bind("<ButtonRelease>", lambda event: self.bt_rack_for_ctrl_released())
        
        self.bt_cylinder_up_for_ctrl.bind("<ButtonRelease>", lambda event: self.bt_cylinder_for_ctrl_released())
        self.bt_cylinder_down_for_ctrl.bind("<ButtonRelease>", lambda event: self.bt_cylinder_for_ctrl_released())
        
    
    def bt_rack_for_ctrl_callbark(self, toggle):
        self.bt_rack_for_ctrl_flag = 1
        speed= int(self.rack_speed_combo_box.get())
        rotation= float(self.rack_rotation_combo_box.get())
        print(f"Rack:{str(toggle)}")
        if toggle == 1:
            self.control.topic_name = "ev3_motor"
            self.control.motor_a_speed = -1 * speed
            self.control.motor_a_rotation = rotation
            self.control.motor_b_speed = 0
            self.control.motor_b_rotation = 0.0
            self.control_pub.publish(self.control)
        else:
            self.control.topic_name = "ev3_motor"
            self.control.motor_a_speed = speed
            self.control.motor_a_rotation = rotation
            self.control.motor_b_speed = 0
            self.control.motor_b_rotation = 0.0
            self.control_pub.publish(self.control)
    
    def bt_rack_for_ctrl_released(self):
        self.bt_rack_for_ctrl_flag = 0

    def bt_cylinder_for_ctrl_callbark(self, toggle):
        self.bt_cylinder_for_ctrl_flag = 1
        speed= int(self.cylinder_speed_combo_box.get())
        rotation= float(self.cylinder_rotation_combo_box.get())
        print(f"Rack:{str(toggle)}")
        if toggle == 1:
            self.control.topic_name = "ev3_motor"
            self.control.motor_a_speed = 0
            self.control.motor_a_rotation = 0.0
            self.control.motor_b_speed = speed
            self.control.motor_b_rotation = rotation
            self.control_pub.publish(self.control)
        else:
            self.control.topic_name = "ev3_motor"
            self.control.motor_a_speed = 0
            self.control.motor_a_rotation = 0.0
            self.control.motor_b_speed = -1 * speed
            self.control.motor_b_rotation = rotation
            self.control_pub.publish(self.control)
            
    def bt_cylinder_for_ctrl_released(self):
        self.bt_cylinder_for_ctrl_flag = 0
        
    def cfg_frame_calibration(self):
        # Label Frameの作成
        # 1列目
        self.frame_calibration_offset = LabelFrame(self.frame_control, text="Rack and Pinion",
                                        width=200, height=200, 
                                        cursor='hand2', labelanchor='n',
                                        borderwidth=1, font = self.pt10_font)
        # 2列目
        self.frame_calibration_offset = LabelFrame(self.frame_control, text="Cylinder",
                                        width=200, height=200, 
                                        cursor='hand2', labelanchor='n',
                                        borderwidth=1, font = self.pt10_font)
    
    def do_something(self):
        if self.bt_rack_for_ctrl_flag == 0:
            self.control.motor_a_speed = 0
            self.control.motor_a_rotation = 0.0
            self.control_pub.publish(self.control)
        if self.bt_cylinder_for_ctrl_flag == 0:
            self.control.motor_b_speed = 0
            self.control.motor_b_rotation = 0.0
            self.control_pub.publish(self.control)
    
    def repeat_action(self):
        self.do_something()
        root.after(1000, self.repeat_action)  # 1秒ごとに処理を繰り返す（単位はミリ秒）
        
    def cfg_frame_measurement(self):
        self.bt_start_for_measurment = Button(self.frame_measurement, text='START', relief='raised', 
                                    bg='pink', width=50, cursor='hand2', font = self.pt10_font, 
                                    command=self.bt_start_for_measurment_callbark)
        
        self.bt_stop_for_measurment = Button(self.frame_measurement, text='STOP', relief='raised', 
                                    bg='sky blue', width=50, cursor='hand2', font = self.pt10_font, 
                                    command=self.bt_stop_for_measurment_callbark)
        
        self.bt_reset_for_measurment = Button(self.frame_measurement, text='RESET', relief='raised', 
                                    bg='pale green', width=50, cursor='hand2', font = self.pt10_font, 
                                    command=self.bt_reset_for_measurment_callbark)
        
        self.bt_start_for_measurment.grid(row=0, column=0, padx=20, pady=5)
        self.bt_stop_for_measurment.grid(row=1, column=0, padx=20, pady=5)
        self.bt_reset_for_measurment.grid(row=2, column=0, padx=20, pady=5)
        
    def bt_reset_for_measurment_callbark(self):
        self.graph_init_data()
        
    def bt_start_for_measurment_callbark(self):
        self.is_update_flag = True
    
    def bt_stop_for_measurment_callbark(self):
        self.is_update_flag = False
        
    # def cfg_frame_Load(self):
    def cfg_frame_graph(self):
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame_graph)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack()
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100, cache_frame_data=False)
        

    def graph_init_data(self):
        self.times = []
        self.values = []
        self.start_time = time.time()
    
    def update_plot(self, frame):
        self.current_time = time.time()
        self.elapsed_time = self.current_time - self.start_time
        
        # 時間が一定の刻み幅を超えた場合、データを更新
        if len(self.times) == 0 or self.elapsed_time - self.times[-1] > self.time_interval:
            self.times.append(self.elapsed_time)
            self.values.append(self.gram_data)

            # 時間が最大表示時間を超えた場合、古いデータを破棄
            if self.times[-1] - self.times[0] > self.max_display_time:
                self.times.pop(0)
                self.values.pop(0)
        
        if self.is_update_flag:
            # データを更新
            self.line.set_data(self.times, self.values)

            # x軸の範囲を設定（時間軸を固定）
            self.ax.set_xlim(max(0, self.elapsed_time - self.max_display_time), self.elapsed_time + 1)
        
        return self.line,

        
    def cfg_frame_result(self):
        self.result_tex = Label(self.frame_result, 
                                height = 100, width = 200,
                                borderwidth = 2,
                                font = self.pt50_font,
                                text = "0000.00")
        self.result_tex.configure(bg="black", fg="white")
        self.result_tex.pack(padx = 25, pady = 25)
        
    def change_text(self, new_text : str):
        self.result_tex.config(text=new_text)  # Labelウィジェットのテキストを更新する
        #self.result_tex.config(state = NORMAL)  # テキストウィジェットを編集可能に変更
        #self.result_tex.insert(END, new_text)  # 新しいテキストを挿入
        #self.result_tex.config(state = DISABLED)  # テキストウィジェットを読み取り専用に戻す
        
    def loadcell_callback(self, msg):
        #if not hasattr(self, 'start_time'):
        #    self.start_time = time.time()
        self.current_time = time.time() - self.start_time
        self.gram_data = float(msg.data)
        if self.gram_data < 0.0:
            self.gram_data = 0.0
        if self.is_update_flag:
            self.change_text("{:.2f}".format(self.gram_data))
    
if __name__ == '__main__':
    root = Tk()
    gui = MainGUI(root)
    root.mainloop()