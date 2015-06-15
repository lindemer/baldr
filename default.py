#! /usr/bin/env morseexec
'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from baldr.builder.robots import Qr
from morse.builder import *

# place the quadrotor on the scene
qr = Qr()
qr.translate(0.0, 0.0, 0.0)
qr.add_default_interface('socket')

# add additional sensors
cam = SemanticCamera()
cam.translate(x=0.3, z=-0.04)
qr.append(cam)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('empty', fastmode = False)
env.set_camera_location([10.0, 10.0, 10.0])
env.set_camera_rotation([1.0, 0.0, 2.0])
env.select_display_camera(cam)
