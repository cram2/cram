import subprocess
import rosnode
import time
import select
import threading

killCommand = 'roslaunch cram_sim_log_generator neem-generation.launch'
#killCommand = 'python blocker.py'



def start_neem_process():
  killProcess = subprocess.Popen(killCommand.split(),bufsize=1, stdout=subprocess.PIPE, universal_newlines=True)
  nice = threading.Timer(360, killProcess.terminate)
  nice.start()

  for line in iter(killProcess.stdout.readline,''):
    nice.cancel() 
    print line.rstrip()
    nice = threading.Timer(360, killProcess.terminate)
    nice.start()


while True:
  try:
    print "start neem process"
    start_neem_process()
  except:
    print "Error occurred"
  

print "Finished"

#output, error = process.communicate()
#print output



    
