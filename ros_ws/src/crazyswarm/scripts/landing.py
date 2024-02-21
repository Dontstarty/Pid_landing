import threading
from pycrazyswarm import Crazyswarm
from geometry_msgs.msg import PoseStamped
import vrpn
import time
import math
import rospy
import logging as logger
from config import LandingConfig
from enum import Enum


class LandingState(Enum):
    INIT = 0
    TAKEOFF = 1
    SEARCH = 2
    GOTO = 3
    TRACK = 4
    FOLLOW = 5
    STATICLAND = 6
    DYNAMICLAND = 7
    RETURN = 8
    FINISH = 9
    END = 10


class PIDController(object):
    def __init__(self):
        self.kp = 0.75
        self.ki = 0.05
        self.kd = 0

        self.error_i = 0.0
        self.error_d = 0.0
        self.error = 0.0
        self.last_error = 0.0


class autonomous_landing:
    """
    autonomous_landing class gets a drone's mission method as a constructor parameter, and when initiated,
    send the drone to the mission. Once the mission is over, the drone automatically performs an autonomous landing
    on a given target, even if the target is moving.
    """

    def __init__(self) -> None:

        self.args = LandingConfig().parser.parse_args()
        self.state = LandingState.INIT
        # set the number of ugv and uav
        self.args.ugv_number = 1
        self.args.uav_number = 1
        # set the ugv_position tracker
        self.ugv_tkr = vrpn.receiver.Tracker('ugv' + str(self.args.ugv_number) + self.args.host)
        self.ugv_tkr.register_change_handler(None,self.get_relative_distance,"position")
        # self.ugv_tkr.register_change_handler(None, self.ugv_pos_update, "position")
        # self.ugv_tkr.register_change_handler("VelHandlerData", self.ugv_vel_update, "velocity")
        # set the uav_positon tracker
        #self.cf1_pose_sub = rospy.Subscriber('/cf2/pose', PoseStamped, self.uav_msg_update)

        # SET VARIABLES
        self.ugv_msg = {"position": [None, None, None], "orientation": [None, None, None],
                        "velocity": [None, None, None], "heading": [None, None, None]}
        self.cf1_msg = {"position": [None, None, None], "orientation": [None, None, None],
                        "velocity": [None, None, None], "heading": [None, None, None]}

        self.time_to_leash = self.args.time_to_leash / self.args.COMMAND_TIME
        self.target_height = self.args.target_height
        self.mission_method = self.args.mission_method
        self.mission_args = self.args.mission_args
        self.landingHeight = self.args.LEASH_HEIGHT - 0.05

        # VARIABLES TO BE CALCULATED
        self.direction_of_target = 0
        self.speed_of_target = 0
        self.b = 0
        self.curr_dist_from_target = [None, None, None]
        self.take_mission = False
        self.tha = None
        self.mission_complete = False

        self.pid_x = PIDController()
        self.pid_y = PIDController()
        self.pid_z = PIDController()

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.cfs = self.allcfs.crazyflies
        self.cf = self.cfs[0]
        self.relative_distance_isupdate = False
        self.takeoff_flag  =False

        self.startTime = None
        self.target_direction = None

        self.count = 0
        self.vel_t_now =None
        self.ugv_msg_last ={"position":[None, None, None]}
        self.ugv_msg_now= {"position":[None, None, None]}

    def ugv_pos_update(self, userdata, t):
        self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2], self.vel_t_now= t['position'][0], \
                t['position'][1], t['position'][2],t['time'].second
        # if self.vel_t_now is not None:
        #     if t["time"].second != self.vel_t_now:
        #         temp = t["time"].second -self.vel_t_now
        #         self.ugv_msg["velocity"][0], self.ugv_msg["velocity"][1] = (t['position'][0] -self.ugv_msg["position"][0])/temp, (t['position'][1]-self.ugv_msg["position"][1])/temp
        # self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2], self.vel_t_now= t['position'][0], \
        #         t['position'][1], t['position'][2],t['time'].second
        # if(self.ugv_msg["velocity"][0] is not None):
        #     print(self.ugv_msg["velocity"])
        # angle = self.to_euler_angles(t['quaternion'][0], t['quaternion'][1], t['quaternion'][2], t['quaternion'][3])
        # self.ugv_msg["orientation"][0], self.ugv_msg["orientation"][1], self.ugv_msg["orientation"][2] = angle["roll"], \
        #     angle["pitch"], angle["yaw"]


    def ugv_msg_get(self, tkr, flag):
        while flag:
            tkr.mainloop()

    def uav_msg_update(self, msg):
        # rospy.loginfo("cf Pose - Position:[{},{},{}]".format(msg.pose.position.x, msg.pose.position.y,
        # msg.pose.position.z))
        self.cf1_msg["position"][0], self.cf1_msg["position"][1], self.cf1_msg["position"][
            2] = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self.cf1_msg["orientation"][0], self.cf1_msg["orientation"][1], self.cf1_msg["orientation"][
            2] = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z

    def takeoff(self):
        z = 1.0
        self.allcfs.takeoff(targetHeight=z, duration=3.0 + z)
        self.timeHelper.sleep(2+ z)

    def land_on_ground(self):
        print("降落至地面")
        self.cfs[0].land(targetHeight=-0.05, duration=4.0)
        self.timeHelper.sleep(4.0)
        # self.cfs[0].stop()

    def land_on_ugv(self):
        print("降落至车上...")
        self.cfs[0].land(targetHeight=0.346, duration=4.0)
        self.timeHelper.sleep(4.0)
        # self.cfs[0].stop()

    def get_relative_distance(self,userdata,t):
        self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2], self.vel_t_now= t['position'][0], \
                t['position'][1], t['position'][2],t['time'].second
        targetx, targety = self.ugv_msg["position"][0], self.ugv_msg["position"][1]
        dronex,droney = self.cf.position()[0],self.cf.position()[1]
        self.curr_dist_from_target[0], self.curr_dist_from_target[1] = (targetx - dronex), (targety - droney)
        self.relative_distance_isupdate = True

    def relative_distance_update(self):
        if self.relative_distance_isupdate:
            print("当前相对距离为：{:.3f},{:.3f}".format(self.curr_dist_from_target[0], self.curr_dist_from_target[1]))
            target_x, target_y = self.compute_position()
            print("期望量：{:.3f},{:.3f}".format(target_x, target_y))
        else:
            print("相对距离还未更新，暂用目标车辆位置")
            target_x, target_y = self.ugv_msg["position"][0], self.ugv_msg["position"][1]
        return target_x, target_y

    def get_direction_of_target(self):
        targetx, targety = self.ugv_msg["position"][0], self.ugv_msg["position"][1]
        dronex,droney = self.cf.position()[0],self.cf.position()[1]
        # dronex, droney = self.cf1_msg["position"][0], self.cf1_msg["position"][1]
        # self.curr_dist_from_target[0] ,self.curr_dist_from_target[1]= (targetx - dronex), (targety -droney)
        # self.curr_dist_from_target = math.dist((targetx, targety), (dronex, droney))
        if self.ugv_msg["velocity"][0]  is not None:
            self.speed_of_target = math.dist((self.ugv_msg["velocity"][0], self.ugv_msg["velocity"][1]), (0, 0))
            self.target_direction = math.atan2(self.ugv_msg["velocity"][1], self.ugv_msg["velocity"][0])
        else:
            self.speed_of_target = 0
            self.target_direction = math.atan2(self.ugv_msg["position"][1], self.ugv_msg["position"][0])


    def compute_position(self):
        self.pid_x.error = self.ugv_msg["position"][0]-self.cf.position()[0]
        self.pid_x.error_d =self.pid_x.error - self.pid_x.last_error
        self.pid_x.error_i +=self.pid_x.error
        self.pid_x.error_i = max(-10.0,min(self.pid_x.error_i,10.0))
        self.pid_x.last_error = self.pid_x.error
        target_x = self.pid_x.kp*self.pid_x.error+self.pid_x.ki*self.pid_x.error_i+self.pid_x.kd*self.pid_x.error_d+self.ugv_msg["position"][0]

        self.pid_y.error = self.ugv_msg["position"][1]-self.cf.position()[1]
        self.pid_y.error_d = self.pid_y.error - self.pid_y.last_error
        self.pid_y.error_i +=self.pid_y.error
        self.pid_y.error_i = max(-10.0,min(self.pid_y.error_i,10.0))
        self.pid_y.last_error = self.pid_y.error
        target_y = self.pid_y.kp*self.pid_y.error+self.pid_y.ki*self.pid_y.error_i+self.pid_y.kd*self.pid_y.error_d+self.ugv_msg["position"][1]

        return target_x, target_y

    def mission(self):
        """drone is doing another mission"""
        logger.info("执行动态降落任务")
        self.mission_method(self.mission_args)
        logger.info("动态降落任务完成")

    def to_euler_angles(self, x, y, z, w):
        angles = {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0}
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

        angles['roll'] = roll
        angles['pitch'] = pitch
        angles['yaw'] = yaw
        return angles

    def get_command(self):
        user_input = input("输入1起飞并执行轨迹,输入其他数字(2-9)退出程序: ")
        self.takeoff_flag = True
        # while True:
        #     if user_input == '1':
        #         # self.takeoff()
        #         # print("飞机当前状态：起飞\n")
        #         self.takeoff_flag = True
        #     else:
        #         print("退出程序")
        #         self.mission_complete = True
    def run(self):
        """
        This is the function that initializes the drone's flight, from take-off to mission to autonomous landing.
        """
        ## ALGO STAGE 1

        if self.state == LandingState.INIT:

            if self.cf.position()[0] is not None and self.ugv_msg["position"][0] is not None:
                self.state = LandingState.TAKEOFF
                print(
                    '车辆当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f},\n 无人机当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f}'.format(
                        self.ugv_msg["position"][0], self.ugv_msg["position"][1], self.ugv_msg["position"][2],
                        self.cf.position()[0],
                        self.cf.position()[1], self.cf.position()[2]))
            else:
                print("无人机与车辆位置丢失，请检查动捕情况\n")
                print(self.cf.position(),self.ugv_msg["position"])
                self.state = LandingState.INIT
        if self.state == LandingState.TAKEOFF:
            if self.takeoff_flag:
                self.takeoff()
                print("飞机当前状态：起飞\n")
                self.state = LandingState.GOTO
            else:
                if self.mission_complete:
                    print("退出程序")
                    self.state = LandingState.END
                else:
                    pass

        if self.state == LandingState.GOTO:
            target_x,target_y = self.relative_distance_update()
            if abs(self.curr_dist_from_target[0]) < 0.05 and abs(self.curr_dist_from_target[1])<0.05 :
                self.state = LandingState.FOLLOW
                print("飞机当前状态：位于平台之上")
                print(self.cf.position())
            else:
                print("飞往平台")
                self.cf.cmdPosition([target_x, target_y, 1], 0)
                self.state = LandingState.GOTO

        if self.state == LandingState.FOLLOW:
            print("当前飞机状态：跟随车辆")
            target_x,target_y = self.relative_distance_update()
            if abs(self.curr_dist_from_target[0]) < 0.05 and abs(self.curr_dist_from_target[1]) < 0.05:
                self.state = LandingState.DYNAMICLAND
            else:
                self.state = LandingState.FOLLOW
                self.cf.cmdPosition([target_x, target_y, self.landingHeight], 0)

        if self.state == LandingState.DYNAMICLAND:
            target_x, target_y = self.relative_distance_update()
            if abs(self.ugv_msg["position"][2]-self.cf.position()[2]) >0.05:
                if abs(self.curr_dist_from_target[0]) < 0.05 and abs(self.curr_dist_from_target[1]) < 0.05:
                    self.state = LandingState.DYNAMICLAND
                    if self.landingHeight>self.target_height:
                        self.landingHeight -=0.02
                        self.cf.cmdPosition([target_x, target_y, self.landingHeight], 0)
                        print("无人机{:d}降落，期望高度为：{:.3f}.当前高度为：{:.3f}".format(self.cf.id, self.landingHeight,self.cf.position()[2]))
                    else:
                        self.cf.cmdStop()
                        self.state = LandingState.FINISH
                else:
                    self.state = LandingState.FOLLOW
            else:
                target_x, target_y = self.relative_distance_update()
                if abs(self.curr_dist_from_target[0]) < 0.05 and abs(self.curr_dist_from_target[1]) < 0.05:
                    self.cf.cmdStop()
                    self.state = LandingState.FINISH
                else:
                    self.cf.cmdPosition([target_x, target_y, self.landingHeight], 0)

        if self.state == LandingState.FINISH:
            print("当前飞机状态：降落完成")
            self.state = LandingState.END
        if self.state == LandingState.RETURN:
            if math.dist((self.cf1_msg["position"][0], self.cf1_msg["position"][1]), (0.75, -0.75, 0)):
                self.cf.cmdPosition([0.75, -0.75, 0], 0)
                print("未在相应时间内完成降落，返航中")
            else:
                self.cf.cmdStop()
                print("返航完成")
                self.state = LandingState.END
        if self.state == LandingState.END:
            self.mission_complete = True
            self.cf.cmdStop()
        if self.startTime - time.time() > 50:
            self.state = LandingState.RETURN
        # self.timeHelper.sleepForRate(36)

    def main(self):
        while self.mission_complete == False :
            self.run()
            self.timeHelper.sleepForRate(18)
if __name__ == '__main__':
    # ROS节点初始化
    landing = autonomous_landing()
    threads = []
    thread = threading.Thread(target=landing.ugv_msg_get, args=(landing.ugv_tkr, True))
    threads.append(thread)
    thread = threading.Thread(target=landing.get_command)
    threads.append(thread)
    for thread in threads:
        thread.start()
    landing.startTime = time.time()
    landing.main()

