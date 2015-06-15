'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
import numpy as np
import math

class FlightPlot():
	def __init__(self, data):
		self.data = data
		max_bound = 50
		x_max = max([abs(i) for i in data['x'] if abs(i) < max_bound])
		y_max = max([abs(i) for i in data['y'] if abs(i) < max_bound])
		z_max = max([abs(i) for i in data['z'] if abs(i) < (2 * max_bound)])
		bound = max([2 * x_max, 2 * y_max, z_max])
		self.make_3d_plot(scale=bound)
		self.make_summary_plot()
		self.make_actuator_plot()
		self.make_rotor_plot()
		plt.show()
	
	def make_3d_plot(self, scale=16):
		fig = plt.figure()
		fig.canvas.set_window_title('3D Flight Trajectory')
		self.ax3d = fig.add_subplot(111, projection='3d')
		self.ax3d.set_xlabel('x axis')
		self.ax3d.set_ylabel('y axis')
		self.ax3d.set_zlabel('z axis')
		self.ax3d.set_xlim(-scale/2,scale/2)
		self.ax3d.set_ylim(-scale/2,scale/2)
		self.ax3d.set_zlim(0,scale)
		self.ax3d.plot(self.data['x'], self.data['y'], self.data['z'], 'b', label='flight path')
		self.ax3d.plot(self.data['xr'], self.data['yr'], self.data['zr'], 'r', label='reference trajectory')
		legend3d = self.ax3d.legend(loc='upper left', fontsize='medium')
		self.plot_3d_patches()

	# calculate rotation matrix
	def get_taitbryan_xyz(self, phi, theta, psi):
		return np.array([	[np.cos(psi) * np.cos(theta), -np.sin(psi) * np.cos(theta), np.sin(theta)],
				 			[np.cos(psi) * np.sin(theta) * np.sin(phi) + np.sin(psi) * np.cos(phi),
								-np.sin(psi) * np.sin(theta) * np.sin(phi) + np.cos(psi) * np.cos(phi),
								-np.cos(theta) * np.sin(phi)],
							[-np.cos(psi) * np.sin(theta) * np.cos(phi) + np.sin(psi) * np.sin(phi),
								np.sin(psi) * np.sin(theta) * np.cos(phi) + np.cos(psi) * np.sin(phi),
					 			np.cos(theta) * np.cos(phi)]	])

	# place the quadrotor representations on the 3d graph
	def plot_3d_patches(self):
		start = 0
		stop = len(self.data['t'])
		step = int((stop - start) / 25)

		# plot a new model after every <step> simulation steps
		for i in np.arange(start, stop, step):
			phi = self.data['phi'][i]
			theta = self.data['theta'][i]
			psi = self.data['psi'][i]
			rotation_matrix = self.get_taitbryan_xyz(phi, theta, psi)

			# make the patch
			self.generate_rotors()
			# differentiate colors to make changes in yaw more visible
			rotor_colors = ['w', 'w', 'w', 'w']
			color_index = 0

			for rotor in [self.rotor1, self.rotor2, self.rotor3, self.rotor4]:
				for vertex in rotor:
					# apply the rotation matrix to the patch
					new_vertex = np.dot(rotation_matrix, vertex)
					vertex[0] = new_vertex[0] + self.data['x'][i]
					vertex[1] = new_vertex[1] + self.data['y'][i]
					vertex[2] = new_vertex[2] + self.data['z'][i]

				# add the patch to the graph
				patch = a3.art3d.Poly3DCollection([rotor])
				patch.set_color(rotor_colors[color_index])
				color_index += 1
				patch.set_edgecolor('k')
				self.ax3d.add_collection3d(patch)

	# create a quadrotor-like shape from four circles
	def generate_rotors(self, r=0.15):
		self.rotor1 = []
		self.rotor2 = []
		self.rotor3 = []
		self.rotor4 = []
		t = np.arange(0, 2 * np.pi, 0.1)
		for s in t:
			# each rotor is represented by a circle with radius r
			g = np.cos(s) * r
			h = np.sin(s) * r
			# place one circle in each quadrant, make all z-coordinates 0 to start
			self.rotor1.append([g + r, h + r, 0])
			self.rotor2.append([g - r, h + r, 0])
			self.rotor3.append([g - r, h - r, 0])
			self.rotor4.append([g + r, h - r, 0])

	def make_summary_plot(self):
		fig = plt.figure()
		fig.canvas.set_window_title('2D Flight Trajectory')

		ax1 = fig.add_subplot(231)
		ax1.plot(self.data['t'], self.data['x'],  'b', label='x')
		ax1.plot(self.data['t'], self.data['xr'], 'r', label='xr')
		ax1.grid()
		ax1.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax1.set_ylabel('coordinate [m]')
		ax1.legend(loc='upper left', fontsize='medium')

		ax2 = fig.add_subplot(232)
		ax2.plot(self.data['t'], self.data['y'],  'b', label='y')
		ax2.plot(self.data['t'], self.data['yr'], 'r', label='yr')
		ax2.grid()
		ax2.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax2.set_ylabel('coordinate [m]')
		ax2.legend(loc='upper left', fontsize='medium')

		ax3 = fig.add_subplot(233)
		ax3.plot(self.data['t'], self.data['z'],  'b', label='z')
		ax3.plot(self.data['t'], self.data['zr'], 'r', label='zr')
		ax3.grid()
		ax3.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax3.set_ylabel('coordinate [m]')
		ax3.legend(loc='upper left', fontsize='medium')

		ax4 = fig.add_subplot(234)
		ax4.plot(self.data['t'], self.data['phi'], 'b', label='phi')
		ax4.grid()
		ax4.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax4.set_xlabel('time (s)')
		ax4.set_ylabel('angle [rad]')
		ax4.legend(loc='upper left', fontsize='medium')

		ax5 = fig.add_subplot(235)
		ax5.plot(self.data['t'], self.data['theta'], 'b', label='theta')
		ax5.grid()
		ax5.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax5.set_xlabel('time (s)')
		ax5.set_ylabel('angle [rad]')
		ax5.legend(loc='upper left', fontsize='medium')

		ax6 = fig.add_subplot(236)
		ax6.plot(self.data['t'], self.data['psi'],  'b', label='psi')
		ax6.plot(self.data['t'], self.data['psir'], 'r', label='psir')
		ax6.grid()
		ax6.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax6.set_xlabel('time (s)')
		ax6.set_ylabel('angle [rad]')
		ax6.legend(loc='upper left', fontsize='medium')

	def make_actuator_plot(self):
		fig = plt.figure()
		fig.canvas.set_window_title('Actuator Values')

		ax1 = fig.add_subplot(221)
		ax1.plot(self.data['t'], self.data['u1'], 'b', label='u1')
		ax1.plot(self.data['t'], self.data['u1r'], 'r', label='u1r')
		ax1.grid()
		ax1.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax1.set_ylabel('thrust [N]')
		ax1.legend(loc='lower right', fontsize='medium')

		ax2 = fig.add_subplot(222)
		ax2.plot(self.data['t'], self.data['u2'], 'b', label='u2')
		ax2.plot(self.data['t'], self.data['u2r'], 'r', label='u2r')
		ax2.grid()
		ax2.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax2.set_ylabel('roll [Nm]')
		ax2.legend(loc='lower right', fontsize='medium')

		ax3 = fig.add_subplot(223)
		ax3.plot(self.data['t'], self.data['u3'], 'b', label='u3')
		ax3.plot(self.data['t'], self.data['u3r'], 'r', label='u3r')
		ax3.grid()
		ax3.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax3.set_xlabel('time (s)')
		ax3.set_ylabel('pitch [Nm]')
		ax3.legend(loc='lower right', fontsize='medium')

		ax4 = fig.add_subplot(224)
		ax4.plot(self.data['t'], self.data['u4'], 'b', label='u4')
		ax4.plot(self.data['t'], self.data['u4r'], 'r', label='u4r')
		ax4.grid()
		ax4.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax4.set_xlabel('time (s)')
		ax4.set_ylabel('yaw [Nm]')
		ax4.legend(loc='lower right', fontsize='medium')

	def make_rotor_plot(self):
		fig = plt.figure()
		fig.canvas.set_window_title('Rotor Speeds')

		ax1 = fig.add_subplot(111)
		ax1.plot(self.data['t'], self.data['omega1'], 'b', label='1')
		ax1.plot(self.data['t'], self.data['omega2'], 'r', label='2')
		ax1.plot(self.data['t'], self.data['omega3'], 'c', label='3')
		ax1.plot(self.data['t'], self.data['omega4'], 'y', label='4')
		ax1.grid()
		ax1.set_xlim(self.data['t'][0], self.data['t'][-1])
		ax1.set_xlabel('time [s]')
		ax1.set_ylabel('rotor speed [rad/s]')
		legend1 = ax1.legend(loc='upper right', fontsize='medium')

