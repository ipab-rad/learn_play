#!/usr/bin/python

import argparse
import sys
from copy import deepcopy
import os

import rospy
from std_msgs.msg import String
import baxter_interface

import learn_play

class LearnPlay(object):
    def __init__(self, limb):
        # Limb is the one that is NOT used by the vision client
        # Grid state
        #  0 - free
        #  1 - occupied
        self._limb = limb
        self._baxter_limb = baxter_interface.Limb(self._limb)
        self._baxter_gripper = baxter_interface.Gripper(self._limb)

        self._no_squares = 8
        self._grid = ([[0 for _i in range(self._no_squares)] 
                      for _j in range(self._no_squares)])
        self._positions_top = {}
        self._positions_top[(0,0)] = {'left_w0': -3.0445683652770996,
                                  'left_w1': -0.93266031796875,
                                  'left_w2': -0.5453301694702148,
                                  'left_e0': -0.09088836157836915,
                                  'left_e1': 0.9054321590148926,
                                  'left_s0': -1.16850986383667,
                                  'left_s1': -0.39768451880493166}

        self._positions_bottom = {}
        self._positions_bottom[(0,0)] = {'left_w0': -3.0457188508666992,
                                         'left_w1': -0.6784030026672364,
                                         'left_w2': -0.4387185048339844,
                                         'left_e0': -0.08168447686157228,
                                         'left_e1': 0.8962282742980958,
                                         'left_s0': -1.1780972437500001,
                                         'left_s1': -0.20286895896606447}

        self._positions_top[(1,1)] = {'left_w0': -1.8465293713073732,
                                      'left_w1': -1.3238254184326173,
                                      'left_w2': -0.9518350777954102,
                                      'left_e0': -1.2490438551086427,
                                      'left_e1': 0.9706263424255371,
                                      'left_s0': -0.6722670795227051,
                                      'left_s1': -0.013805827075195313}
        self._positions_bottom[(1,1)] ={'left_w0': -1.8549662656311037,
                                        'left_w1': -1.0749370358825685,
                                        'left_w2': -0.8387039948181153,
                                        'left_e0': -1.113670050732422,
                                        'left_e1': 0.9169370149108887,
                                        'left_s0': -0.6860729065979004,
                                        'left_s1': 0.11543205415649414}

        self._positions_top[(2,2)] = {'left_w0': -1.3430001782592775,
                                      'left_w1': -1.4484613573059082,
                                      'left_w2': -1.1301603441833497,
                                      'left_e0': -1.523242920629883,
                                      'left_e1': 1.3238254184326173,
                                      'left_s0': -0.4498398655334473,
                                      'left_s1': 0.26614566639404297}
        self._positions_bottom[(2,2)] = {'left_w0': -1.8446118953247073,
                                         'left_w1': -0.983665179107666,
                                         'left_w2': -1.0657331511657715,
                                         'left_e0': -0.996704015789795,
                                         'left_e1': 1.2624661869873048,
                                         'left_s0': -0.5648884244934083,
                                         'left_s1': -0.021475731005859377}

        self._positions_top[(3,3)] = {'left_w0': -1.191136080432129,
                                      'left_w1': -1.4845099057800295,
                                      'left_w2': -1.2862428891723634,
                                      'left_e0': -1.491029324121094,
                                      'left_e1': 1.621417690942383,
                                      'left_s0': -0.2822524646484375,
                                      'left_s1': 0.34936412404174805}
        self._positions_bottom[(3,3)] = {'left_w0': -1.6708885712951662,
                                         'left_w1': -0.853276812286377,
                                         'left_w2': -1.3192234760742187,
                                         'left_e0': -0.9299758515930177,
                                         'left_e1': 1.5938060367919924,
                                         'left_s0': -0.4391020000305176,
                                         'left_s1': -0.10891263581542969}

        self._positions_top[(4,4)] = {'left_w0': -1.181548700518799,
                                      'left_w1': -1.4680196123291016,
                                      'left_w2': -1.3123205625366212,
                                      'left_e0': -1.417781741583252,
                                      'left_e1': 1.813932279602051,
                                      'left_s0': -0.1307718620178223,
                                      'left_s1': 0.3520485904174805}
        self._positions_bottom[(4,4)] = {'left_w0': -1.6566992490234376,
                                      'left_w1': -0.9127185677490235,
                                      'left_w2': -1.379815717126465,
                                      'left_e0': -0.93266031796875,
                                      'left_e1': 1.738767221081543,
                                      'left_s0': -0.36086897993774414,
                                      'left_s1': -0.13575729957275393}

        self._positions_top[(5,5)] = {'left_w0': -1.2229661817443849,
                                      'left_w1': -1.55162156517334,
                                      'left_w2': -1.3272768752014161,
                                      'left_e0': -1.410111837652588,
                                      'left_e1': 1.9569759879089357,
                                      'left_s0': -0.014189322271728517,
                                      'left_s1': 0.34859713364868167}
        self._positions_bottom[(5,5)] = {'left_w0': -1.4756895162597656,
                                         'left_w1': -0.928825366003418,
                                         'left_w2': -1.3805827075195314,
                                         'left_e0': -0.9552865345642091,
                                         'left_e1': 1.9247623914001466,
                                         'left_s0': -0.194432064642334,
                                         'left_s1': -0.09357282795410157}

        self._positions_top[(6,6)] = {'left_w0': -1.1029321852294922,
                                      'left_w1': -1.5109710743408205,
                                      'left_w2': -1.4135632944213867,
                                      'left_e0': -1.295063278692627,
                                      'left_e1': 2.218136216748047,
                                      'left_s0': 0.19059711267700197,
                                      'left_s1': 0.3443786864868164}
        self._positions_bottom[(6,6)] = {'left_w0': -1.1397477240966798,
                                         'left_w1': -1.140514714489746,
                                         'left_w2': -1.4039759145080568,
                                         'left_e0': -1.040038972998047,
                                         'left_e1': 2.084679888354492,
                                         'left_s0': 0.12386894848022462,
                                         'left_s1': 0.1844611895324707}

        self._positions_top[(7,7)] = {'left_w0': -1.094878786102295,
                                      'left_w1': -1.4354225206237794,
                                      'left_w2': -1.4396409677856445,
                                      'left_e0': -1.179631224536133,
                                      'left_e1': 2.335102251690674,
                                      'left_s0': 0.32213596508789066,
                                      'left_s1': 0.2711311039489746}
        self._positions_bottom[(7,7)] = {'left_w0': -1.054995285662842,
                                         'left_w1': -1.259398225415039,
                                         'left_w2': -1.404742904901123,
                                         'left_e0': -1.01166032845459,
                                         'left_e1': 2.197427476135254,
                                         'left_s0': 0.2692136279663086,
                                         'left_s1': 0.2427524594055176}

        self._baxter_gripper.calibrate()


        
        # for i in xrange(len(self.grid)):
        #     print self.grid[i]
            
        self.game_off  = False
        self._turn = 0 # 1, 2 = baxter, human
        self._no_pieces = 0 # Number of pieces on board

    def pick_piece(self,
 pos = 0):
        if pos == 0:
            self._default_bottom = {'left_w0': -1.1746457869812013,
                                    'left_w1': -1.2432914271606446,
                                    'left_w2': -0.836403023638916,
                                    'left_e0': -1.2110778306518555,
                                    'left_e1': 1.7721313031799317,
                                    'left_s0': 0.3271214026428223,
                                    'left_s1': 0.28685440700683595}

            self._default_top = {'left_w0': -1.2965972594787598,
                                 'left_w1': -1.3541215389587404,
                                 'left_w2': -0.8601797258239746,
                                 'left_e0': -1.3472186254211427,
                                 'left_e1': 1.8821944245849611,
                                 'left_s0': 0.39231558605346684,
                                 'left_s1': 0.21782527163085938}

            self._default_middle = {'left_w0': -1.785937130255127,
                                    'left_w1': -1.3782817363403321,
                                    'left_w2': -1.2425244367675783,
                                    'left_e0': -1.5090535983581543,
                                    'left_e1': 1.8361750010009767,
                                    'left_s0': -0.05944175546264649,
                                    'left_s1': -0.22396119477539064}


            i = None
            print rospy.Time.now(), "Picking piece"
            self._baxter_limb.move_to_joint_positions(self._default_middle)
            self._baxter_limb.move_to_joint_positions(self._default_top)
            self._baxter_gripper.open()
            # close
            self._baxter_limb.move_to_joint_positions(self._default_bottom)
            rospy.sleep(0.5)
            self._baxter_gripper.close()
            rospy.sleep(0.5)
            self._baxter_limb.move_to_joint_positions(self._default_top)
            self._baxter_limb.move_to_joint_positions(self._default_middle)

    def place_piece(self, pos = 0):
        print rospy.Time.now(), "Placing piece to", pos

        if (pos != 0):
            self._baxter_limb.move_to_joint_positions(
                self._positions_top[pos])
            # closed
            self._baxter_limb.move_to_joint_positions(
                self._positions_bottom[pos])
            self._baxter_gripper.open()
            rospy.sleep(0.5)
            self._baxter_limb.move_to_joint_positions(
                self._positions_top[pos])
        else:
            print "Specify place!"
                
        
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--limb', dest='limb', choices=['left', 'right'],
                        required=True, help='limb to control')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_learn_play_%s' % (args.limb,)) 
    lp = LearnPlay(args.limb)
    while(1):
        for i in range(8):
            lp.pick_piece()
            lp.place_piece((i,i))
            
    
    # 1 get data from vision
    # 2 start arms stuff
    # 3 wait for user pieces (3?)
    # 4 calculates next move
    # 5 make move
    # 6 goto 3

if __name__ == "__main__":
    main()
