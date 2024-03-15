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
from ros_ws.src.crazyswarm.msg import FullState.msg



class LandingState(Enum):
    INIT = 0
    READY = 1
    TAKEOFF = 2
    TRACK = 3
    FOLLOW = 4
    LAND = 5
    RETURN = 6
    FINISH = 7
    END = 8


class Pid(object):
    def __init__(self):
        self.kp = 0.75
        self.ki = 0.05
        self.kd = 0

        self.error_i = 0.0
        self.error_d = 0.0
        self.error = 0.0
        self.last_error = 0.0

        self.sature = 10

    def update(self,error):
        self.error = error
        self.error_d =self.error - self.last_error
        self.error_i +=self.error
        self.error_i = max(-self.sature,min(self.error_i,self.sature))
    def pd_compute(self,error):
        self.update(error)
        return self.kp*self.error+self.kd*self.error_d
    def pi_compute(self,error):
        self.update(error)
        return self.kp*self.error+self.ki*self.error_i
    def p_compute(self,error):
        self.update(error)
        return self.kp*self.error
    def pid_compute(self,error):
        self.update(error)
        return self.kp * self.error + self.ki * self.error_i+self.kd*self.error_d

class Controller(Pid):
    def __init__(self):
        super().__init__()
        self.x = Pid()
        self.y = Pid()
        self.z = Pid()




class AutonomousLanding:
    """
    autonomous_landing class gets a drone's mission method as a constructor parameter, and when initiated,
    send the drone to the mission. Once the mission is over, the drone automatically performs an autonomous landing
    on a given target, even if the target is moving.
    """

    def __init__(self,cf,ugv) -> None:

        self.args = LandingConfig().parser.parse_args()
        self.state = LandingState.INIT
        self.cf = cf
        self.ugv = ugv
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        # set the ugv_position tracker
        self.ugv_tkr =  vrpn.receiver.Tracker(self.ugv + self.args.host)
        self.ugv_tkr.register_change_handler(None,self.get_ugv_msg,"position")

        # SET VARIABLES
        self.ugv_msg = FullState()
        self.cf_msg = FullState()
        self.global_time = time.time()
        self.desire = FullState()
        self.tracking_e = self.args.tracking_error


        self.hover_height = self.args.TAKE_OFF_HEIGHT
        self.target_height = self.args.target_height
        self.mission_args = self.args.mission_args
        self.landingHeight = self.args.LEASH_HEIGHT - 0.05
        self.yaw = self.args.yaw

        # SET FLAG
        self.command_time = time.time()
        self.relative_distance_isupdate = False
        self.takeoff_flag  =False
        self.take_mission = False
        self.mission_complete = False
        self.ugv_speed_isupdate = False

        # VARIABLES TO BE CALCULATED
        self.curr_dist_from_target = [None, None, None]
        self.curr_vel_from_target = [None, None, None]

        # SET Controller
        self.velocity_pid = Controller()
        self.position_pid = Controller()


    #创建车辆信息监听器
    def tkr_listen(self, tkr,flag):
        while flag:
            tkr.mainloop()

    # 起飞
    def takeoff(self):
        self.allcfs.takeoff(targetHeight=self.target_height, duration=3.0 + self.target_height)
        self.timeHelper.sleep(3+ self.target_height)



    #获取车辆状态信息
    def get_ugv_msg(self,t):
        temp = time.time() - self.global_time
        if temp >0.5:
            self.global_time = time.time()
            self.ugv_msg.twist.linear.x ,self.ugv_msg.twist.linear.y,self.ugv_msg.twist.linear.z = \
                (t['position'][0] -self.ugv_msg.pose.position.x)/temp , (t['position'][1] -self.ugv_msg.pose.position.y)/temp, \
                    (t['position'][2] -self.ugv_msg.pose.position.z)/temp
            self.ugv_speed_isupdate = True
        else:
            pass
        self.ugv_msg.pose.position.x,self.ugv_msg.pose.position.y,self.ugv_msg.pose.position.z = \
            t['position'][0], t['position'][1], t['position'][2]
        angles = self.to_euler_angles(t['quaternion'][0], t['quaternion'][1], t['quaternion'][2])
        self.ugv_msg.pose.orientation.x,self.ugv_msg.pose.orientation.y,self.ugv_msg.pose.orientation.z = \
            angles["roll"],angles["pitch"],angles["yaw"]
        self.curr_dist_from_target[0],self.curr_dist_from_target[1]= (self.ugv_msg.pose.position.x - self.cf.position()[0]), \
            (self.ugv_msg.pose.position.y -self.cf.position()[1])
        self.relative_distance_isupdate = True


    #计算相对位置
    def relative_distance_update(self):
        if self.relative_distance_isupdate:
            self.relative_distance_isupdate = False
            print("当前相对距离为：{:.3f},{:.3f}".format(self.curr_dist_from_target[0], self.curr_dist_from_target[1]))
            self.desire.pose.position.x, self.desire.pose.position.y = self.compute_position_desire(self.curr_dist_from_target)
            print("期望量：{:.3f},{:.3f}".format( self.desire.pose.position.x, self.desire.pose.position.y))
        else:
            print("相对距离还未更新，暂用目标车辆位置")
            self.desire.pose.position.x, self.desire.pose.position.y =  self.ugv_msg.pose.position.x,self.ugv_msg.pose.position.y

    # 计算相对速度
    def relative_velocity_update(self):
        if self.ugv_speed_isupdate:
            print("当前速度：{:.3f},{:.3f}".format( self.ugv_msg.twist.linear.x ,self.ugv_msg.twist.linear.y))
            self.ugv_speed_isupdate = False
            self.desire.twist.linear.x, self.desire.twist.linear.y = self.compute_velocity_desire(self.curr_vel_from_target)
            print("期望量：{:.3f},{:.3f}".format( self.desire.twist.linear.x, self.desire.twist.linear.y ))
        else:
            print("相对距离还未更新，暂用目标车辆位置")
            self.desire.twist.linear.x, self.desire.twist.linear.y = self.ugv_msg.twist.linear.x ,self.ugv_msg.twist.linear.y

    # 计算期望位置
    def compute_position_desire(self,error):
        target_x = self.position_pid.pid_compute(error[0])+self.ugv_msg.pose.position.x
        target_y= self.position_pid.pid_compute(error[1]) + self.ugv_msg.pose.position.y

        return target_x, target_y

    #计算期望速度
    def compute_velocity_desire(self,error):
        target_x = self.position_pid.pid_compute(error[0]) + self.ugv_msg.twist.linear.x
        target_y = self.position_pid.pid_compute(error[1]) + self.ugv_msg.twist.linear.y

        return target_x, target_y

    #四元数计算欧拉角
    def to_euler_angles(self, x, y, z, w):
        angles = {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0}
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(2 * (w * y - z * z))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

        angles['roll'] = roll
        angles['pitch'] = pitch
        angles['yaw'] = yaw
        return angles

    #打印信息
    def message_print(self):
        print(
            '车辆当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f},\n 无人机当前坐标为：\n x:{:.3f},y:,{:.3f},z:{:.3f}'.format(
                self.ugv_msg.pose.position.x, self.ugv_msg.pose.position.y, self.ugv_msg.pose.position.z,
                self.cf.position()[0],
                self.cf.position()[1], self.cf.position()[2]))

    #降落流程
    def run(self):
        """
        This is the function that initializes the drone's flight, from take-off to mission to autonomous landing.
        """
        ## 获取相对距离
        self.relative_distance_update()
        #初始状态
        if self.state == LandingState.INIT:
            #判断动捕是否获取到无人机与车辆信息
            if self.cf.position()[0] is not None and self.ugv_msg.pose.position.z != 0:
                self.state = LandingState.READY
                self.message_print()
            else:
                print("无人机与车辆位置丢失，请检查动捕情况\n")
                print(self.cf.position(),self.ugv_msg.pose.position)
                self.state = LandingState.INIT
        # 起飞指令
        if self.state == LandingState.READY:
            user_input = input("输入1起飞并执行轨迹,输入其他数字(2-9)退出程序: ")
            if user_input == '1':
                self.state = LandingState.TAKEOFF
                self.message_print()
            else:
                self.state = LandingState.END
                print("退出程序")
        # 起飞状态
        if self.state == LandingState.TAKEOFF:
            self.takeoff()
            print("飞机当前状态：起飞\n")
            self.state = LandingState.TRACK
        # 追踪态，xy两轴相对距离大于tracking_e,持续发送目标点
        if self.state == LandingState.TRACK:
            if abs(self.curr_dist_from_target[0]) < self.tracking_e and abs(self.curr_dist_from_target[1])<self.tracking_e :
                self.state = LandingState.FOLLOW
                print("飞机当前状态：位于平台之上")
                print(self.cf.position())
            else:
                print("飞往平台")
                self.cf.cmdPosition([self.desire.pose.position.x, self.desire.pose.position.y, self.hover_height], self.yaw)
                self.state = LandingState.TRACK

        # 跟随态，xy两轴相对距离小于tracking_e,开始降落
        if self.state == LandingState.FOLLOW:
            print("当前飞机状态：跟随车辆")
            if abs(self.curr_dist_from_target[0]) < self.tracking_e and abs(self.curr_dist_from_target[1]) < self.tracking_e:
                self.state = LandingState.LAND
            else:
                self.state = LandingState.FOLLOW
                self.cf.cmdPosition([self.desire.pose.position.x,  self.desire.pose.position.y, self.landingHeight], self.yaw)
        # 降落态，z轴相对距离小于tracking_e,降落
        if self.state == LandingState.LAND:
            if abs(self.ugv_msg["position"][2]-self.cf.position()[2]) >self.tracking_e:
                if abs(self.curr_dist_from_target[0]) < self.tracking_e and abs(self.curr_dist_from_target[1]) < self.tracking_e:
                    self.state = LandingState.LAND
                    if self.landingHeight>self.target_height:
                        self.landingHeight -=0.02
                        self.cf.cmdPosition(
                            [self.desire.pose.position.x, self.desire.pose.position.y, self.landingHeight], self.yaw)
                        print("无人机{:d}降落，期望高度为：{:.3f}.当前高度为：{:.3f}".format(self.cf.id, self.landingHeight,self.cf.position()[2]))
                    else:
                        self.cf.cmdStop()
                        self.state = LandingState.FINISH
                else:
                    self.state = LandingState.FOLLOW
            else:
                if abs(self.curr_dist_from_target[0]) < self.tracking_e and abs(self.curr_dist_from_target[1]) < self.tracking_e:
                    self.cf.cmdStop()
                    self.state = LandingState.FINISH
                else:
                    self.cf.cmdPosition([self.desire.pose.position.x, self.desire.pose.position.y, self.landingHeight], self.yaw)

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
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs = allcfs.crazyflies
    landing = AutonomousLanding(cfs[0],"ugv1")
    threads = []
    thread = threading.Thread(target=landing.tkr_listen, args=(landing.ugv_tkr, True))
    threads.append(thread)
    for thread in threads:
        thread.start()
    landing.main()

