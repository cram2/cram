import subprocess
import rosnode
import time
import select
import threading

import rospy
from world_control_msgs.srv import *

killCommand = 'roslaunch cram_sim_log_generator neem-generation.launch'
#killCommand = 'python blocker.py'



def start_neem_process():
  reset_level = rospy.ServiceProxy('/UnrealSim/reset_level', ResetLevel)
  rsp = reset_level()
  killProcess = subprocess.Popen(killCommand.split(),bufsize=1, stdout=subprocess.PIPE, universal_newlines=True)
  nice = threading.Timer(360, killProcess.terminate)
  nice.start()

  for line in iter(killProcess.stdout.readline,''):
    nice.cancel()
    print line.rstrip()
    nice = threading.Timer(360, killProcess.terminate)
    nice.start()

rospy.wait_for_service('/UnrealSim/reset_level')
while True:
  try:
    print "start neem process"
    start_neem_process()
  except:
    print "Error occurred"


print "Finished"

#output, error = process.communicate()
#print output
