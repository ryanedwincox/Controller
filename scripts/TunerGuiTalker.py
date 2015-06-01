#!/usr/bin/env python 

from TunerGui import TunerGui
import rospy
from std_msgs.msg import String


def mainDrive():
	if not rospy.is_shutdown():
		sliders = gui.getConstants()
		string = ''		
		for x in sliders:
			string += ("," + str(x))

		pub.publish(string)

	gui.after(1, mainDrive) # loops the mainDrive method

if __name__ == "__main__":
	rospy.init_node('pidConstants', anonymous=True)

	pub = rospy.Publisher('pidConstants', String, queue_size=10)
	gui = TunerGui()	
	gui.after(1, mainDrive)
	gui.mainloop()
