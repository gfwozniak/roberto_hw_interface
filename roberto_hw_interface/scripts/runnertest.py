#!/usr/bin/env python

import rospy
import threading
from roberto_api import Roberto

if __name__ == '__main__':
    try:
        robot = Roberto()
        thread1 = threading.Thread(target=robot.setActuatorPosition, args=(60,))
        thread2 = threading.Thread(target=robot.setAugerVelocity, args=(50,))
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()
    except rospy.ROSInterruptException:
        pass