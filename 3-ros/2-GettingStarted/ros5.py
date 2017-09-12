from xmlrpclib import ServerProxy
import os
master = ServerProxy(os.environ['ROS_MASTER_URI'])
print master.getSystemState('/')