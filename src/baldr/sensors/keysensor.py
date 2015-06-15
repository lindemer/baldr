'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import logging; logger = logging.getLogger('morse.' + __name__)
import morse.core.sensor
from morse.core import blenderapi
from morse.helpers.components import add_data

class KeySensor(morse.core.sensor.Sensor):

	_name = 'KeySensor'
	_short_desc = 'Provides a data stream of keyboard events from the Blender API.'

	add_data('up',    False, 'boolean', 'up arrow key sensor')
	add_data('down',  False, 'boolean', 'down arrow key sensor')
	add_data('left',  False, 'boolean', 'left arrow key sensor')
	add_data('right', False, 'boolean', 'right arrow key sensor')
	add_data('i',	  False, 'boolean', 'i key sensor')
	add_data('j',	  False, 'boolean', 'j key sensor')
	add_data('k',	  False, 'boolean', 'k key sensor')
	add_data('l',	  False, 'boolean', 'l key sensor')

	def __init__(self, obj, parent=None):
		logger.info("%s initialization" % obj.name)
		morse.core.sensor.Sensor.__init__(self, obj, parent)
		logger.info('Component initialized')

	def default_action(self):
		keyboard = blenderapi.keyboard()
		is_actived = blenderapi.input_active()

		if keyboard.events[blenderapi.UPARROWKEY] == is_actived:
			self.local_data['up'] = True
		else: self.local_data['up'] = False

		if keyboard.events[blenderapi.DOWNARROWKEY] == is_actived:
			self.local_data['down'] = True
		else: self.local_data['down'] = False

		if keyboard.events[blenderapi.LEFTARROWKEY] == is_actived:
			self.local_data['left'] = True
		else: self.local_data['left'] = False

		if keyboard.events[blenderapi.RIGHTARROWKEY] == is_actived:
			self.local_data['right'] = True
		else: self.local_data['right'] = False

		if keyboard.events[blenderapi.IKEY] == is_actived:
			self.local_data['i'] = True
		else: self.local_data['i'] = False

		if keyboard.events[blenderapi.JKEY] == is_actived:
			self.local_data['j'] = True
		else: self.local_data['j'] = False

		if keyboard.events[blenderapi.KKEY] == is_actived:
			self.local_data['k'] = True
		else: self.local_data['k'] = False

		if keyboard.events[blenderapi.LKEY] == is_actived:
			self.local_data['l'] = True
		else: self.local_data['l'] = False

