#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import subprocess
import rosparam
from std_msgs.msg import String,Bool
import pygame
from rois_ros.srv import *
from rois_ros.msg import *
import actionlib
import threading
from datetime import datetime
from rois_ros.srv import *

#eventにも挑戦



class Person_DetectionService():
    def __init__(self):

        #初期化できていないためコンポーネントの状態をUNINITIALIZEDにする
        self.comp_state = "UNINITIALIZED"
        print(self.comp_state)        
        
        #コンポーネント名を設定する
        self.comp_ref = "Person_Detection"

        #コンポーネントの状態確認のサービス設定
        state_name = '/get_state/' + self.comp_ref
        self.state_service = rospy.Service(state_name, component_status, self.component_status)



        #Command用のアクション通信の設定
        exe_name = '/execute/' + self.comp_ref
        self.server = actionlib.SimpleActionServer(exe_name, executeAction, self.execute, False)
        self.server.start()
        rospy.loginfo(f"{exe_name} Service is ready.")
        
        #Event用のサービス通信の設定
        exe_name = '/Subscribe/' + self.comp_ref
        self.sub_server = rospy.Service(exe_name, subscribe_set, self.subs)
        rospy.loginfo(f"{exe_name} Service is ready.")

        # #パラメータ操作用のサービス通信の設定
        self.detect_human = rospy.ServiceProxy('/detection', detection)  
        self.detect_human1 = rospy.ServiceProxy('/start', JudgeParam)  
        self.detect_human2 = rospy.ServiceProxy('/stop', JudgeParam)  

        #Thread用の設定
        self.state = "idle"
        self.detected_thread = None

        self.num = 0


        #イベント通知するためのトピック通信の設定
        self.pub = rospy.Publisher('/event_persondetec', notify_persondetect , queue_size=1)

        #コマンド終了の通知用のトピックの設定
        self.pub1 = rospy.Publisher('/completed_event', completed , queue_size=1)

        self.detect_sub = rospy.Subscriber('/judge_param', Bool , self.decb)
        #イベント通知のトピックを送信するか判断する変数の設定
        self.SEND_DETECT = False         #person_detectedメソッド用

        #準備できたらコンポーネントの状態をREADYにする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをREADY状態にする




    #コンポーネントが持つパラメータを設定するためのset_parameter()
    def set_parameter(self, s_req):
        print("set parameter")



    def get_parameter(self,g_req):
        print("get_parameter")


    def execute(self, goal):
        self.comp_state = "BUSY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')  #コンポーネントをBUSY状態にする

        self.command = goal.command_name
        rospy.loginfo("Received command: %s", self.command)

        self.feedback = executeFeedback()
        self.result = executeResult()

        if self.command == "start":
            self.start()    
            
        elif self.command == "stop":
            self.stop()

        elif self.command == "suspend":
            self.suspend()

        elif self.command == "resume":
            self.resume()

        else:
            rospy.loginfo("No valid command received.")
            self.result.success = "False"
            self.server.set_aborted(self.result)



    def start(self):
        print("start")

        self.state = "playing"

        self.result.success = "True"
        self.server.set_succeeded(self.result)

        self.detected_thread = threading.Thread(target=self.detection)
        self.detected_thread.start()


    def stop(self):
        print("stop")
        # if self.state == "playing":
        #     self.state = "stopped"
        #     print("音声認識は停止しました")

        #     self.feedback.status = "playing stopped."
            
        #     pub_data = completed()
        #     pub_data.command_id = "start"
        #     pub_data.status = "stopped"
        #     self.pub.publish(pub_data)
            
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)

        #     self.comp_state = "READY"
        #     rospy.loginfo(f'Componemt status: {self.comp_state}') 
        # else:
        #     rospy.logwarn("No active playing.")
        #     self.feedback.status = "No active goal to stop."
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)


    def suspend(self):
        print("suspend")
        # if self.state == "playing":
        #     pygame.mixer.music.pause()
        #     self.state = "suspended"
        #     self.feedback.status = "play suspended."
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)
        # else:
        #     rospy.logwarn("Cannot suspend; not playing.")
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)


    def resume(self):
        print("resume")
        # if self.state == "suspended":
        #     pygame.mixer.music.unpause()
        #     self.state = "playing"
        #     self.feedback.status = "Playing resumed."
        #     self.result.success = "True"
        #     self.server.set_succeeded(self.result)

        #     if not self.recognized_thread or not self.recognized_thread.is_alive():
        #         self.recognized_thread = threading.Thread(target=self.recognize)
        #         self.recognized_thread.start()
        # else:
        #     rospy.logwarn("No previous goal to resume.")
        #     self.result.success = "False"
        #     self.server.set_aborted(self.result)


    def subs(self, goal):
        self.event = goal.event_type
        
        #イベントメソッド名を指定
        if self.event == "person_detected":
            #トピックを送信するように設定
            self.SEND_TEXT = True
            #返答
            return subscribe_setResponse("True")

        #該当するイベントメソッド名がない場合
        else:
            rospy.loginfo("No valid command received.")
            return subscribe_setResponse("False")

    def decb(self,msg):
        if not msg.data:
            print("return")
            return
        else:
            self.num = msg.data
            print(self.num)


    #音声認識
    def detection(self):
        rospy.loginfo("Monitoring playback.")
        if self.state == "playing":

            response = self.detect_human1()
            # response = self.detect_human("start")
             
            print(response)

            while self.num <1:
                print("detect...")
                rospy.sleep(0.1)

            completed_time= rospy.get_time()
            
            self.completed_time =str(datetime.fromtimestamp(completed_time))

            response = self.detect_human2()
            
            self.on_detection_complete(self.completed_time,self.num)
            # if response.result == "success":
                # self.person_detected(response.timestamp,response.number)
                               
                # return



    def person_detected(self,timestamp,number):
        print(timestamp,number)
        
        pub_data = notify_persondetect()
        pub_data.event_type = "person_detected"
        pub_data.timestamp = timestamp
        pub_data.number = 1

        # pub_data = notify_persondetect()
        # pub_data.event_type = "person_detected"
        # pub_data.timestamp = timestamp
        # pub_data.number = number

        print(pub_data)


        self.pub.publish(pub_data)
        
        
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}')
       


    #完了したら呼び出される関数
    def on_detection_complete(self,timestamp,number):
        rospy.loginfo("recognize completed successfully.")

        #認識が完了した時間と認識結果を通知するメソッド
        self.person_detected(timestamp,number)

        #READY状態にする
        self.comp_state = "READY"
        rospy.loginfo(f'Componemt status: {self.comp_state}') 



    def component_status(self, req):
        # 現在の状態を返答
        if (req.component_name == self.comp_ref):
            rospy.loginfo("Current state requested: %s", self.comp_state)
            return component_statusResponse(self.comp_state)
        else:
            pass

    def run(self):
        rospy.loginfo("Service node is running...")
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node('Person_Detection')
    print("time")
    service = Person_DetectionService()
    service.run()
