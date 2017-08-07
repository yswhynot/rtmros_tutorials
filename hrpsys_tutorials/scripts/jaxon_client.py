#!/usr/bin/env python

import sys

from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP
import argparse

#
class JaxonConfigurator(HrpsysConfigurator):

    def goPos(self, x, y, th):
        '''!@brief Walk to the goal position and orientation. Returns without waiting for whole steps to be executed.
        @param i_x[m], i_y[m], and i_th[deg] are goal x-y-position and z-orientation from the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goPos(x, y, th)

    def goVelocity(self, vx, vy, vth):
        '''!@brief Walk at the desired velocity. If the robot is stopping, the robot starts stepping. Returns without waiting for whole steps to be executed.
        @param i_vx[m/s], i_vy[m/s], and i_vth[deg/s] are velocity in the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goVelocity(vx, vy, vth)

    def goStop(self):
        '''!@brief Stop stepping.
        @param
        @return true if set successfully, false otherwise'''
        self.abc_svc.goStop()

class JAXON(JaxonConfigurator):
    rtclist = [
        ['seq', "SequencePlayer"],
        ['sh', "StateHolder"],
        ['fk', "ForwardKinematics"],
        ['ic', "ImpedanceController"],
        ['el', "SoftErrorLimiter"],
        # ['co', "CollisionDetector"],
        ['sc', "ServoController"],
        ['log', "DataLogger"],
    ]

    def init(self, robotname="Jaxon", url=""):
        print(self.configurator_name + "initialize")

    def getRTCList(self):
        rtclist = self.rtclist
        return rtclist

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='jaxon command line interpreters')
    parser.add_argument('--use-collision', help='install collision detector', action='store_true', default=False)
    args, unknown = parser.parse_known_args()
    unknown = [u for u in unknown if u[:2] != '__'] # filter out ros arguments
    print("args: ")
    print(args)
    print("unknown: ")
    print(unknown)

    hcf = JAXON()
    # hcf = JaxonConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable

    # initialize if we have arguments
    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
        # hcf.waitForRTCManagerAndRoboHardware(robotname=args.robot)
        hcf.init(args.robot, args.modelfile)
    else:
        hcf.findComps()

    # add collision detector
    if args.use_collision:
        print("do use collision")
        hcf.rtclist.append(['co', "CollisionDetector"])

    # if auto balancer is not started yet
    if hcf.abc_svc.getAutoBalancerParam()[1].controller_mode != OpenHRP.AutoBalancerService.MODE_ABC:
        hcf.setJointAngles(
            [0.0, 0.0, -20.0, 40.0, -20.0, 0.0,
             0.0, 0.0, -20.0, 40.0, -20.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0,
             0.0, +40.0, -20.0, -5.0, -80.0, 0.0, 0.0, -20.0,
             0.0, +40.0, +20.0, +5.0, -80.0, 0.0, 0.0, -20.0,
             0,0,0,0], 2)
        hcf.waitInterpolation()
        hcf.startAutoBalancer()
    
