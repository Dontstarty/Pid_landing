import argparse
import time


class LandingConfig():
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser = argparse.ArgumentParser(
            prog='autonomous landing',
            description='uav tracking of ugv and then landing',
            epilog='Text at the bottom of help')

        self.parser.add_argument('--host', type=str, default='@192.168.66.131',
                                 help='des of vrpn)')
        self.parser.add_argument('--ugv_number', type=int, default=1,
                                 help='the number of the ugvs')
        self.parser.add_argument('--uav_number', type=int, default=4,
                                 help='the number of the uavs')
        self.parser.add_argument('--LEASH_DISTANCE', type=float, default=0.3)
        self.parser.add_argument('--LEASH_HEIGHT', type=float, default=1.0)
        self.parser.add_argument('--TAKE_OFF_HEIGHT', type=float,default=1.0)
        self.parser.add_argument('--COMMAND_TIME', type=float, default=1.0)
        self.parser.add_argument('--time_to_leash', type=float, default=20)
        self.parser.add_argument('--target_height', type=float, default=0.30)
        self.parser.add_argument('--mission_method', type=callable, default=time.sleep)
        self.parser.add_argument('--mission_args', type=int, default=5)

