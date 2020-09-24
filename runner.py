

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
# import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String


# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 30
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


def run():
    
    sumoBinary = checkBinary('sumo')
    # else:
    #     sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/cross.sumocfg",'--fcd-output', 'fcd.data'])

    """execute the TraCI control loop"""
    import traci.constants as tc
    vehID = "left_0"
    # traci.vehicle.subscribe(vehID, (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))

    step = 0
    # we start with phase 2 where EW has green
    traci.trafficlight.setPhase("0", 2)
    pos = []

    # try:
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if traci.trafficlight.getPhase("0") == 2:
            # we are not already switching  
            if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
                # there is a vehicle from the north, switch
                traci.trafficlight.setPhase("0", 3)
            else:
                # otherwise try to keep green for EW
                traci.trafficlight.setPhase("0", 2)
        step += 1

        # if not rospy.is_shutdown():
        rospy.loginfo(traci.vehicle.getPosition3D(vehID))
        pub.publish(str(traci.vehicle.getPosition3D(vehID)))

    # except Exception:
    #     pass

        # print(traci.vehicle.getSubscriptionResults(vehID))
    # pos = np.array(pos)
    # plt.scatter(pos[:,0], pos[:,1])
    # plt.show()
    traci.close()
    sys.stdout.flush()



def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

start_run = False
def callback(data):
    global start_run
    start_run = True

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    # if options.nogui:

    # run()
    rospy.Subscriber('start', String, callback)
    while True:
        if start_run:
            run()
            break
        rospy.sleep(0.1)



    

# sumo -c data/cross.sumocfg --fcd-output fcd.data --tripinfo-output tripinfo.xml --fcd-output.geo