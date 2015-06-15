'''
	Developed by Samuel Tanner Lindemer 2015
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from morse.builder import *
from baldr.builder.actuators import Transporter
from baldr.builder.sensors import KeySensor

class Qr(Robot):
	def __init__(self, name=None):

		# qr.blend is located in the data/robots directory
		Robot.__init__(self, 'baldr/robots/qr.blend', name)
		self.properties(classpath = "baldr.robots.qr.Qr")

		# create a new Transporter actuator
		self.transporter = Transporter()
		self.append(self.transporter)

		# create a new KeySensor sensor
		self.keysensor = KeySensor()
		self.append(self.keysensor)
