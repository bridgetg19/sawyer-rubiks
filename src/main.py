import argparse

import rospy

import time

from read_rubiks import *
from color_detect import *
from kociemba_stuff import *

from dictionary import poses as poses

import intera_interface

from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
    Cuff,
)

testKociembaString = "R U R’ U R U2 R’ U"


# Based on keyboard gripper control example
class GripperConnect(object):
    """
    Connects wrist button presses to gripper open/close commands.

    Uses the Navigator callback feature to make callbacks to connected
    action functions when the button values change.
    """

    def __init__(self, arm, lights=True):
        """
        @type arm: str
        @param arm: arm of gripper to control
        @type lights: bool
        @param lights: if lights should activate on cuff grasp
        """
        self._arm = arm
        # inputs
        self._cuff = Cuff(limb=arm)
        # connect callback fns to signals
        try:
            self._gripper = get_current_gripper_interface()
            self._is_clicksmart = isinstance(self._gripper, SimpleClickSmartGripper)

            if self._is_clicksmart:
                if self._gripper.needs_init():
                    self._gripper.initialize()
            else:
                if not (self._gripper.is_calibrated() or
                        self._gripper.calibrate() == True):
                    raise
            self._cuff.register_callback(self._close_action,
                                         '{0}_button_upper'.format(arm))
            self._cuff.register_callback(self._open_action,
                                         '{0}_button_lower'.format(arm))

            rospy.loginfo("{0} Cuff Control initialized...".format(
                          self._gripper.name))
        except:
            self._gripper = None
            self._is_clicksmart = False
            msg = ("{0} Gripper is not connected to the robot."
                   " Running cuff-light connection only.").format(arm.capitalize())
            rospy.logwarn(msg)

    # Open gripper
    def _open_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper open triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', False)
    
    # Close gripper
    def _close_action(self, value):
        if value and self._gripper.is_ready():
            rospy.logdebug("gripper close triggered")
            if self._is_clicksmart:
                self._gripper.set_ee_signal_value('grip', True)

# Writes the current joint positions to the dictionary file. Used to record joint positions 
def write_joint_positions(joints, limb):
    f = open('dictionary.py', 'a')
    f.write("{\n")
    for j in range(7):
        f.write(f"\tjoints[{j}]: {limb.joint_angle(joints[j])},\n")
    f.write("}")

# Prints the current joint positions. Used to record joint positions to move to
def print_joint_positions(joints, limb):
    for j in range(len(joints)):
        print(f"joint",j," ",limb.joint_angle(joints[j]))
    print("")

# Rotates the cube up
def rotate_up(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["u1"])
    limb.move_to_joint_positions(poses["u2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["u3"])
    limb.move_to_joint_positions(poses["u4"])
    limb.move_to_joint_positions(poses["u5"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["u6"])
    limb.move_to_joint_positions(poses["rest"])

# Rotates the cube down 
def rotate_down(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["d1"])
    limb.move_to_joint_positions(poses["d2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["d3"])
    limb.move_to_joint_positions(poses["d4"])
    limb.move_to_joint_positions(poses["d5"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["d6"])
    limb.move_to_joint_positions(poses["rest"])

# Rotates the cube to the right
def rotate_right(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["r1"])
    limb.move_to_joint_positions(poses["r2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["r3"])
    limb.move_to_joint_positions(poses["r4"])
    limb.move_to_joint_positions(poses["r5"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["r6"])
    limb.move_to_joint_positions(poses["rest"])

# Rotates the cube to the left
def rotate_left(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["l1"])
    limb.move_to_joint_positions(poses["l2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["l3"])
    limb.move_to_joint_positions(poses["l4"])
    limb.move_to_joint_positions(poses["l5"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["l6"])
    limb.move_to_joint_positions(poses["rest"])

# Turns the bottom face 90 degrees clockwise
def clockwise_turn(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["c1"])
    limb.move_to_joint_positions(poses["c2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["c3"])
    limb.move_to_joint_positions(poses["c4"])
    limb.move_to_joint_positions(poses["c5"])
    limb.move_to_joint_positions(poses["c6"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["rest"])

# Turns the bottom face 90 degrees counter clockwise
def c_clockwise_turn(limb, grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["cc1"])
    limb.move_to_joint_positions(poses["cc2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["cc3"])
    limb.move_to_joint_positions(poses["cc4"])
    limb.move_to_joint_positions(poses["cc5"])
    limb.move_to_joint_positions(poses["cc6"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["rest"])

# From the front face, rotates the cube to the back face
def rotate_to_back(limb, grip):
    rotate_right(limb,grip)
    rotate_right(limb,grip)

def test_move(limb, grip):
    rotate_left(limb,grip)

# From the front back face, rotates the cube to the front face
def rotate_to_front(limb, grip):
    rotate_left(limb,grip)
    rotate_left(limb,grip)

# Turns the bottom face 180 degrees
def turn_180(limb,grip):
    grip[0]._open_action(True)
    limb.move_to_joint_positions(poses["rest"])
    limb.move_to_joint_positions(poses["2c1"])
    limb.move_to_joint_positions(poses["2c2"])
    grip[0]._close_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["2c3"])
    limb.move_to_joint_positions(poses["2c4"])
    grip[0]._open_action(True)
    time.sleep(1)
    limb.move_to_joint_positions(poses["2c5"])
    limb.move_to_joint_positions(poses["rest"])

# Given a reference to limb and grip, uses the arm to move the cube to various positions and takes pictures of each side
def scan_cube(limb, grip):
    rotate_up(limb, grip)
    rotate_up(limb, grip)
    #take picture of up
    start_video_single("img/face_1.png")
    print("Taking picture of up")
    rotate_down(limb, grip)
    rotate_right(limb, grip)
    #take picture of right
    print("Taking picture of right")
    start_video_single("img/face_2.png")
    rotate_left(limb, grip)
    #take picture of front
    print("Taking picture of front")
    start_video_single("img/face_3.png")
    rotate_down(limb, grip)
    #take picture of down
    print("Taking picture of down")
    start_video_single("img/face_4.png")
    rotate_up(limb, grip)
    rotate_left(limb, grip)
    #take picture of left
    print("Taking picture of left")
    start_video_single("img/face_5.png")
    rotate_left(limb, grip)
    #take picture of back
    print("Taking picture of back")
    start_video_single("img/face_6.png")
    rotate_left(limb, grip)
    rotate_left(limb, grip)
    rotate_down(limb,grip)
    print("Done scanning")

# Overall solve function. Scans the cube, detects colors, and solve 
def start_solve(side, args, valid_limbs):
    limb = intera_interface.Limb(side)

    # Create gripper object
    arms = (args.gripper,) if args.gripper != 'all_limbs' else valid_limbs[:-1]
    grip_ctrls = [GripperConnect(arm, args.lights) for arm in arms]

    # Set move speed to 25%
    limb.set_joint_position_speed(.25)

    # Scan all sides of the cube
    scan_cube(limb, grip_ctrls)

    print("Detecting colors")
    #take images from the video and determine the colors
    cell_centers = get_cell_centers()
    f1, h1 = color_detect(cv2.imread('img/face_1.png'), cell_centers)
    f2, h2 = color_detect(cv2.imread('img/face_2.png'), cell_centers)
    f3, h3 = color_detect(cv2.imread('img/face_3.png'), cell_centers)
    f4, h4 = color_detect(cv2.imread('img/face_4.png'), cell_centers)
    f5, h5 = color_detect(cv2.imread('img/face_5.png'), cell_centers)
    f6, h6 = color_detect(cv2.imread('img/face_6.png'), cell_centers)
    print("Detected all colors")

    #replace center tile of face 1 with "W" bc of logo
    f1[4] = 'W'

    #concatenate all the faces together
    rubiks_cube = list_to_string(f1) + list_to_string(f2) + list_to_string(f3) + list_to_string(f4) + list_to_string(f5) + list_to_string(f6)
    
    #translate string from WYROGB to UDRLFB
    kociemba_input = translate_string(rubiks_cube)
    print(rubiks_cube)
    print(kociemba_input)

    #send to kociemba algorithm and solve rubiks cube
    kociemba_sol = kociemba(kociemba_input)
    kociemba_move(kociemba_sol, limb, grip_ctrls)

# Using the kociemba solve string, call all moves needed to solve the rubiks cube
def kociemba_move(solve_string, limb, grip):
    moves = solve_string.split(" ")
    i = 0
    for move in moves:
        print(f"Move {i} of {len(moves)}, \"{move}\"")
        #Move to required face
        if move[0] == "R":
            rotate_right(limb, grip)
        elif move[0] == "U":
            rotate_up(limb, grip)
        elif move[0] == "L":
            rotate_left(limb, grip)
        elif move[0] == "D":
            rotate_down(limb, grip)
        elif move[0] == "B":
            rotate_to_back(limb, grip)

        #Rotate face
        if (len(move) > 1):
            #counter clockwise or 2 rotations
            if move[1] == "'":
                c_clockwise_turn(limb, grip)
            elif move[1] == "2":
                turn_180(limb,grip)
        #Rotate clockwise
        else:
            clockwise_turn(limb, grip)

        #Move back from face to neutral 
        if move[0] == "R":
            rotate_left(limb, grip)
        elif move[0] == "U":
            rotate_down(limb, grip)
        elif move[0] == "L":
            rotate_right(limb, grip)
        elif move[0] == "D":
            rotate_up(limb, grip)
        elif move[0] == "B":
            rotate_to_front(limb, grip)

# Program that uses the sawyer robotic arm to solve a rubiks cube
# Based on the keyboard gripper control example provided by sawyer
def main():
    epilog = ""
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument('-g', '--gripper', dest='gripper', default=valid_limbs[0],
                        choices=[valid_limbs],
                        help='gripper limb to control (default: both)')
    parser.add_argument('-n', '--no-lights', dest='lights',
                        action='store_false',
                        help='do not trigger lights on cuff grasp')
    parser.add_argument('-v', '--verbose', dest='verbosity',
                        action='store_const', const=rospy.DEBUG,
                        default=rospy.INFO,
                        help='print debug statements')
    args = parser.parse_args(rospy.myargv()[1:])
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('sdk_gripper_cuff_control_{0}'.format(args.gripper),
                    log_level=args.verbosity)
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    
    start_solve(args.limb, args, valid_limbs)
    print("Done.")


if __name__ == '__main__':
    main()
