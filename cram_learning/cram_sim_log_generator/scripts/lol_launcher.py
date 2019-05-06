import subprocess
import rosnode
import time

bashCommand = 'roslaunch json_prolog json_prolog_single.launch'
killCommand = 'pkill -f json_prolog'

isStarted = False

print "Killing JSON Prolog ...."
killProcess = subprocess.Popen(killCommand.split(), stdout=subprocess.PIPE)
time.sleep(5)
print 'Start json prolog ...'
subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

while not isStarted:
  isStarted = rosnode.rosnode_ping('/json_prolog', 1)

time.sleep(5)
print 'Started'
#output, error = process.communicate()
#print output



    
