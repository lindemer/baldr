'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import logging; logger = logging.getLogger('morse.' + __name__)
import morse.core.actuator
import numpy as np
from morse.core import mathutils
from morse.helpers.components import add_data

class Transporter(morse.core.actuator.Actuator):
	'''
	This actuator transports the robot to the absolute position with 
	respect to the origin of the Blender coordinate reference. 
	Angles are expected in radians.

	The orientation of the robot is calculated with an XYZ Tait-Bryan
	rotation matrix.
	'''
	_name = 'Transporter'
	_short_desc = 'Moves the parent robot and sets its orientation.'

	add_data('x', 'initial robot X position', 'float',
	      'X coordinate of the destination, in meter')
	add_data('y', 'initial robot Y position', 'float',
	      'Y coordinate of the destination, in meter')
	add_data('z', 'initial robot Z position', 'float',
	     'Z coordinate of the destination, in meter')
	add_data('phi', 'Initial robot roll', 'float',
		      'Rotation of the robot around X axis, in radians')
	add_data('theta', 'Initial robot pitch', 'float',
		      'Rotation of the robot around Y axis, in radians')
	add_data('psi', 'Initial robot yaw', 'float',
		    'Rotation of the robot around Z axis, in radians')

	def __init__(self, obj, parent=None):
		logger.info('%s initialization' % obj.name)
		# Call the constructor of the parent class
		morse.core.actuator.Actuator.__init__(self, obj, parent)

		self.orientation = self.bge_object.orientation.to_euler('XYZ')

		self.local_data['phi'] = self.orientation.x
		self.local_data['theta'] = self.orientation.y
		self.local_data['psi'] = self.orientation.z
		
		pose3d = self.robot_parent.position_3d

		self.local_data['x'] = pose3d.x
		self.local_data['y'] = pose3d.y
		self.local_data['z'] = pose3d.z

		logger.info('Component initialized')
	
	def get_taitbryan_xyz(self, phi, theta, psi):
		return [[np.cos(psi) * np.cos(theta), -np.sin(psi) * np.cos(theta), np.sin(theta)],
	 		[np.cos(psi) * np.sin(theta) * np.sin(phi) + np.sin(psi) * np.cos(phi),
				-np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi),
				-np.cos(theta) * np.sin(phi)],
			[-np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi),
				np.sin(psi) * np.sin(theta) * np.cos(phi) + np.cos(psi) * np.sin(phi),
	 			np.cos(theta) * np.cos(phi)]]

	def default_action(self):
		''' Change the parent robot pose. '''

		# New parent position
		position = mathutils.Vector((self.local_data['x'],
				             self.local_data['y'],
				             self.local_data['z']))

		# New parent orientation
		phi   = self.local_data['phi']
		theta = self.local_data['theta']
		psi   = self.local_data['psi']

		rotation_matrix = self.get_taitbryan_xyz(phi, theta, psi)
		self.robot_parent.force_pose(position, rotation_matrix)

