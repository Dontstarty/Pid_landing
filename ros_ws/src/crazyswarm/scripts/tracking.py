# args.des
import threading
from pycrazyswarm import Crazyswarm
from geometry_msgs.msg import PoseStamped

import argparse
import vrpn
import time
import math
import rospy

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
cfs = allcfs.crazyflies
cfs['position']

interrupt_trajectory = False
ugv_position,ugv_orientation = [None,None,None],[None,None,None]
cf1_position,cf2_orientation = [None,None,None],[None,None,None]

uav_position_update_flag = False
ugv_position_update_flag = False

position = [2.75,-1.75,0.380]



def takeoff():
    Z = 1
    allcfs.takeoff(targetHeight=Z, duration=3.0 + Z)
    timeHelper.sleep(3 + Z)


def land():
    print("降落中...")
    for cf in cfs:
        cf.land(targetHeight=0.30, duration=4.0)
    timeHelper.sleep(4.0)

def land_ugv():
    print("降落中...")
    for cf in cfs:
        cf.land(targetHeight=-0.05, duration=4.0)
    timeHelper.sleep(4.0)
def ugv_position_update(userdata,t):
    global ugv_position,ugv_orientation
    # print("handle_tracker_position\tSensor {:d} is at ({:.3f},{:.3f},{:.3f}) at time {}".format(
    #     t['sensor'],
    #     t['position'][0], t['position'][1], t['position'][2],
    #     t['time'])
    # )
    ugv_position[0], ugv_position[1], ugv_position[2] = t['position'][0], t['position'][1], t['position'][2]
    # ugv_position[0],ugv_position[1],ugv_position[2] =  msg.pose.position.x, msg.pose.position.y ,msg.pose.position.z

def ugv_position_get(tkr,flag):
    while flag:
        tkr.mainloop()


def cf1_position_update(msg):
    global cf1_position,uav_position_update_flag
    #rospy.loginfo("cf Pose - Position:[{},{},{}]".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    cf1_position[0], cf1_position[1], cf1_position[2] = msg.pose.position.x, msg.pose.position.y ,msg.pose.position.z

def tracking():
    global cfs, cf1_position,ugv_position
    takeoff()
    Z = 1
    tracking_time = time.time()
    count = 0
    while True:
        # rospy.loginfo("cf Pose - Position:[{},{},{}]".format(cf1_position[0], cf1_position[1], cf1_position[2]))
        # rospy.loginfo("ugv Pose - Position:[{},{},{}]".format(ugv_position[0], ugv_position[1], ugv_position[2]))
        if (time.time() - tracking_time) < 10:
            if math.dist((cf1_position[0],cf1_position[1]),(ugv_position[0],ugv_position[1]))>0.05 :
                print(math.dist((cf1_position[0], cf1_position[1]), (ugv_position[0],ugv_position[1])))
                cfs[0].cmdPosition([ugv_position[0],ugv_position[1],1],0)
                timeHelper.sleepForRate(18)
                count+=1
                print("靠近目标点,距离为:"+str(math.dist((cf1_position[0], cf1_position[1]), (ugv_position[0],ugv_position[1])))+"\n")
            else:
                print('开始降落')
                if abs(cf1_position[2]-ugv_position[2])>0.10:
                    Z-=0.01
                    cfs[0].cmdPosition((ugv_position[0], ugv_position[1], Z),0)
                    print('缓慢降落')
                else:
                    land()
                    break
        else:
            land()
            print(count)
            break


def tracking_test():
    global cfs,cf1_position
    takeoff()
    Z = 1
    tracking_time = time.time()
    count = 0
    while True:
        #rospy.loginfo("cf Pose - Position:[{},{},{}]".format(cf1_position[0], cf1_position[1], cf1_position[2]))
        if (time.time() -tracking_time)<10:
            if math.dist((cf1_position[0],cf1_position[1]),(2.75,-0.75))>0.10 :
                print(math.dist((cf1_position[0],cf1_position[1]),(2.75,-0.75)))
                cfs[0].cmdPosition([2.75,-0.75,1],0)
                timeHelper.sleepForRate(18)
                count+=1
                print("靠近目标点")
            else:
                if abs(cf1_position[2]-0.40)>0.1 :
                    if Z  >0.80:
                        print('缓慢降落')
                        Z-=0.02
                        cfs[0].cmdPosition((2.75,-0.75,Z),0)
                        timeHelper.sleepForRate(20)
                    else:
                        print('直接降落')
                        land_ugv()
                        break
        else:
            land()
            print(count)
            break



if __name__ == "__main__":
    threads = []
    parser = argparse.ArgumentParser(
        prog='tracker_client',
        description='Connects to a tracker and prints positionr reports',
        epilog='Text at the bottom of help')
    parser.add_argument('--des', type=str, default='@192.168.66.131',
                            help='des of device)')
    parser.add_argument('--ugv_number', type=int, default=1,
                            help='the number of the ugvs')
    parser.add_argument('--uav_number', type=int, default=4,
                            help='the number of the ugvs')
    args = parser.parse_args()
    args.ugv_number = 1
    ugv_tkr = vrpn.receiver.Tracker('ugv' + str(args.ugv_number) + args.des)
    ugv_tkr.register_change_handler(None, ugv_position_update, "position");
    rospy.Subscriber('/cf2/pose',PoseStamped,cf1_position_update)
    thread = threading.Thread(target=ugv_position_get, args=(ugv_tkr, True))
    thread.start()
    tracking()
    thread.join()


