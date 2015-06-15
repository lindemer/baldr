'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from mpl_toolkits.mplot3d.art3d import Line3D
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
import pickle
import snap

class WindowManager:
	def __init__(self, filename, power=10.00, tilt=None, guess=5.00):
		self.filename = filename
		self.power = power
		self.tilt = tilt
		self.guess = guess

		self.xy_figure = plt.figure()
		self.xz_figure = plt.figure()
		self.xyz_figure = plt.figure()
		self.wppsi_figure = plt.figure()

		self.xy_axis = self.xy_figure.add_subplot(111)
		self.xz_axis = self.xz_figure.add_subplot(111)
		self.xyz_axis = self.xyz_figure.add_subplot(111, projection='3d')
		self.wppsi_axis = self.wppsi_figure.add_subplot(111)

		self.xy_figure.canvas.set_window_title('XY Top-Down View')
		self.xz_figure.canvas.set_window_title('XZ Side View')
		self.xyz_figure.canvas.set_window_title('3D Projection')
		self.wppsi_figure.canvas.set_window_title('Yaw View')

		self.xy_background = self.xy_figure.canvas.copy_from_bbox(self.xy_axis.bbox)
		self.xz_background = self.xz_figure.canvas.copy_from_bbox(self.xz_axis.bbox)
		self.xyz_background = self.xyz_figure.canvas.copy_from_bbox(self.xyz_axis.bbox)
		self.wppsi_background = self.wppsi_figure.canvas.copy_from_bbox(self.wppsi_axis.bbox)

		self.xy_figure.canvas.mpl_connect('key_press_event', self.key_press_callback)
		self.xz_figure.canvas.mpl_connect('key_press_event', self.key_press_callback)
		self.xyz_figure.canvas.mpl_connect('key_press_event', self.key_press_callback)
		self.wppsi_figure.canvas.mpl_connect('key_press_event', self.key_press_callback)

		self.xy_axis.set_xlim(-10, 10)
		self.xy_axis.set_ylim(-10, 10)
		self.xy_axis.set_xlabel('x [m]')
		self.xy_axis.set_ylabel('y [m]')
		self.xz_axis.set_xlim(-10, 10)
		self.xz_axis.set_ylim(-2, 18)
		self.xz_axis.set_xlabel('x [m]')
		self.xz_axis.set_ylabel('z [m]')
		self.xyz_axis.set_xlim(-10, 10)
		self.xyz_axis.set_ylim(-10, 10)
		self.xyz_axis.set_zlim(0, 20)
		self.wppsi_axis.set_xlim(0, 5)
		self.wppsi_axis.set_ylim(-2 * np.pi, 2 * np.pi)
		self.wppsi_axis.set_xlabel('waypoint #')
		self.wppsi_axis.set_ylabel('psi [rad]')
		self.waypoints = MoveableWaypoints(self.xy_axis, self.xz_axis, self.wppsi_axis)
		plt.show()

	def key_press_callback(self, event):
		if event.key == 'shift':		# Run optimization algorithm.
			x_wp, y_wp, z_wp, psi_wp = self.waypoints.get_data()
			x = snap.Trajectory1D(x_wp)
			y = snap.Trajectory1D(y_wp)
			z = snap.Trajectory1D(z_wp, der=2)
			psi = snap.Trajectory1D(psi_wp, der=2)

			self.xy_lines = []
			self.xz_lines = []
			self.xyz_lines = []
			self.xyz_vectors = [[], []]
			self.xyz_axis.clear()
			self.xyz_figure.canvas.draw()

			print('Running optimization routine...')
			self.trajectory = snap.QrPath(x, y, z, psi, power=self.power, tilt=self.tilt, guess=self.guess)
			T = self.trajectory.optimize()

			tdata = np.arange(0, sum(T), 0.1)
			xdata = [x(t) for t in tdata]
			ydata = [y(t) for t in tdata]
			zdata = [z(t) for t in tdata]
			self.xy_lines.append(Line2D(xdata, ydata, linestyle='-', marker='', color='b'))
			self.xz_lines.append(Line2D(xdata, zdata, linestyle='-', marker='', color='b'))
			self.xyz_lines.append(Line3D(xdata, ydata, zdata, linestyle='-', marker='', color='k'))

			self.xy_axis.add_line(self.xy_lines[-1])
			self.xy_figure.canvas.restore_region(self.xy_background)
			self.xy_axis.draw_artist(self.xy_lines[-1])
			self.xy_figure.canvas.blit(self.xy_axis.bbox)

			self.xz_axis.add_line(self.xz_lines[-1])
			self.xz_figure.canvas.restore_region(self.xz_background)
			self.xz_axis.draw_artist(self.xz_lines[-1])
			self.xz_figure.canvas.blit(self.xz_axis.bbox)

			self.xyz_axis.add_line(self.xyz_lines[-1])
			self.xyz_figure.canvas.restore_region(self.xyz_background)
			self.xyz_axis.draw_artist(self.xyz_lines[-1])
			self.xyz_figure.canvas.blit(self.xyz_axis.bbox)

			wp_t = [0]
			for t in T:
				wp_t.append(wp_t[-1] + t)

			for t in wp_t:
				self.xyz_vectors[0].append(Line3D([x(t, d=0), x(t, d=0) + x(t, d=1)], [y(t, d=0), y(t, d=0) + y(t, d=1)], [z(t, d=0), z(t, d=0) + z(t, d=1)], \
					color='m', lw=2, marker='o', markeredgecolor='m'))
				self.xyz_axis.add_line(self.xyz_vectors[0][-1])
				self.xyz_figure.canvas.restore_region(self.xyz_background)
				self.xyz_axis.draw_artist(self.xyz_vectors[0][-1])
				self.xyz_figure.canvas.blit(self.xyz_axis.bbox)

				self.xyz_vectors[1].append(Line3D([x(t, d=0), x(t, d=0) + x(t, d=2)], [y(t, d=0), y(t, d=0) + y(t, d=2)], [z(t, d=0), z(t, d=0) + z(t, d=2)], \
					color='c', lw=2, marker='o', markeredgecolor='c'))
				self.xyz_axis.add_line(self.xyz_vectors[1][-1])
				self.xyz_figure.canvas.restore_region(self.xyz_background)
				self.xyz_axis.draw_artist(self.xyz_vectors[1][-1])
				self.xyz_figure.canvas.blit(self.xyz_axis.bbox)

				self.xyz_lines.append(Line3D(x(t), y(t), z(t), color='w', linestyle='', marker='o', markeredgecolor='k'))
				self.xyz_axis.add_line(self.xyz_lines[-1])
				self.xyz_figure.canvas.restore_region(self.xyz_background)
				self.xyz_axis.draw_artist(self.xyz_lines[-1])
				self.xyz_figure.canvas.blit(self.xyz_axis.bbox)

		elif event.key == 'z':		# Save trajectory to file.
			datafile = open(self.filename, 'wb')
			pickle.dump(self.trajectory, datafile)
			datafile.close()
			print('Trajectory saved to data file.', end='\n\n')

		elif event.key == 't':		# Toggle waypoint adjustment.
			self.waypoints.toggle = not self.waypoints.toggle
			print('Waypoint adjustment toggled.', end='\n\n')

class MoveableWaypoints:
	def __init__(self, xy_axis, xz_axis, wppsi_axis):
		self.xy_axis = xy_axis
		self.xy_line = Line2D([0.0], [0.0], linestyle='--', color='r', marker='o', markerfacecolor='w', markeredgecolor='k', animated=True)
		self.xy_axis.add_line(self.xy_line)
		self.xy_canvas = self.xy_line.figure.canvas
		self.xy_canvas.mpl_connect('draw_event', self.xy_draw_callback)
		self.xy_canvas.mpl_connect('button_press_event', self.button_press_callback)
		self.xy_canvas.mpl_connect('button_release_event', self.button_release_callback)
		self.xy_canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)

		self.xz_axis = xz_axis
		self.xz_line = Line2D([0.0], [0.0], linestyle='--', color='r', marker='o', markerfacecolor='w', markeredgecolor='k', animated=True)
		self.xz_axis.add_line(self.xz_line)
		self.xz_canvas = self.xz_line.figure.canvas
		self.xz_canvas.mpl_connect('draw_event', self.xz_draw_callback)
		self.xz_canvas.mpl_connect('button_press_event', self.button_press_callback)
		self.xz_canvas.mpl_connect('button_release_event', self.button_release_callback)
		self.xz_canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)

		self.wppsi_axis = wppsi_axis
		self.wppsi_line = Line2D([0.0], [0.0], linestyle='--', color='r', marker='o', markerfacecolor='w', markeredgecolor='k', animated=True)
		self.wppsi_axis.add_line(self.wppsi_line)
		self.wppsi_canvas = self.wppsi_line.figure.canvas
		self.wppsi_canvas.mpl_connect('draw_event', self.wppsi_draw_callback)
		self.wppsi_canvas.mpl_connect('button_press_event', self.button_press_callback)
		self.wppsi_canvas.mpl_connect('button_release_event', self.button_release_callback)
		self.wppsi_canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)

		self.toggle = True			# Waypoint adjustment activation
		self.epsilon = 5			# Click sensitivity [pixels]
		self._ind = None			# Selected index
		self._psi = False			# Psi plot selected boolean

	def xy_draw_callback(self, *args):
		self.xy_background = self.xy_canvas.copy_from_bbox(self.xy_axis.bbox)
		self.xy_axis.draw_artist(self.xy_line)
		self.xy_canvas.blit(self.xy_axis.bbox)

	def xz_draw_callback(self, *args):
		self.xz_background = self.xz_canvas.copy_from_bbox(self.xz_axis.bbox)
		self.xz_axis.draw_artist(self.xz_line)
		self.xz_canvas.blit(self.xz_axis.bbox)

	def wppsi_draw_callback(self, *args):
		self.wppsi_background = self.wppsi_canvas.copy_from_bbox(self.wppsi_axis.bbox)
		self.wppsi_axis.draw_artist(self.wppsi_line)
		self.wppsi_canvas.blit(self.wppsi_axis.bbox)

	def get_ind_under_point(self, event):
		if event.inaxes is self.xy_axis:
			xy = np.asarray(self.xy_line.get_data()).T				# Get matrix of line data
			last = len(xy)											# Index of new datapoint
			xyt = self.xy_line.get_transform().transform(xy)		# Transform matrix to fig coords
			xt, yt = xyt[:, 0], xyt[:, 1]							# Separate into two column vectors x and y
			d = np.sqrt((xt - event.x)**2 + (yt - event.y)**2)		# Make new column vector of distances to click
			indseq = np.nonzero(np.equal(d, np.amin(d)))[0]			# Make a list of indicies ordered by closeness
			ind = indseq[0]											# Closest click
			if d[ind] >= self.epsilon:
				self.new_waypoint(event)
			else: return ind
			return last

		elif event.inaxes is self.xz_axis:
			xz = np.asarray(self.xz_line.get_data()).T				# Get matrix of line data
			last = len(xz)											# Index of new datapoint
			xzt = self.xz_line.get_transform().transform(xz)		# Transform matrix to fig coords
			xt, zt = xzt[:, 0], xzt[:, 1]							# Separate into two column vectors x and y
			d = np.sqrt((xt - event.x)**2 + (zt - event.y)**2)		# Make new column vector of distances to click
			indseq = np.nonzero(np.equal(d, np.amin(d)))[0]			# Make a list of indicies ordered by closeness
			ind = indseq[0]											# Closest click
			if d[ind] >= self.epsilon:
				self.new_waypoint(event)
			else: return ind
			return last

		elif event.inaxes is self.wppsi_axis:
			wppsi = np.asarray(self.wppsi_line.get_data()).T			# Get matrix of line data
			wppsit = self.wppsi_line.get_transform().transform(wppsi)	# Transform matrix to fig coords
			wpt, psit = wppsit[:, 0], wppsit[:, 1]						# Separate into two column vectors x and y
			d = np.sqrt((wpt - event.x)**2 + (psit - event.y)**2)		# Make new column vector of distances to click
			indseq = np.nonzero(np.equal(d, np.amin(d)))[0]				# Make a list of indicies ordered by closeness
			ind = indseq[0]												# Closest click
			if d[ind] <= self.epsilon: return ind
			else: return None

	def button_press_callback(self, event):
		if event.inaxes == None: return
		if self.toggle == False: return
		self._ind = self.get_ind_under_point(event)

	def button_release_callback(self, event):
		self._ind = None

	def motion_notify_callback(self, event):
		if self._ind is None: return
		if event.inaxes is None: return

		if event.inaxes is self.xy_axis:
			x, y = event.xdata, event.ydata
			xy_data = self.xy_line.get_data()
			xz_data = self.xz_line.get_data()
			xy_data[0][self._ind] = x
			xy_data[1][self._ind] = y
			xz_data[0][self._ind] = x
			self.xy_line.set_data(xy_data)
			self.xz_line.set_data(xz_data)

			self.xy_canvas.restore_region(self.xy_background)
			self.xy_axis.draw_artist(self.xy_line)
			self.xy_canvas.blit(self.xy_axis.bbox)
			self.xz_canvas.restore_region(self.xz_background)
			self.xz_axis.draw_artist(self.xz_line)
			self.xz_canvas.blit(self.xz_axis.bbox)

		elif event.inaxes is self.xz_axis:
			x, z = event.xdata, event.ydata
			xz_data = self.xz_line.get_data()
			xz_data[0][self._ind] = x
			xz_data[1][self._ind] = z
			xy_data = self.xy_line.get_data()
			xy_data[0][self._ind] = x
			self.xz_line.set_data(xz_data)
			self.xy_line.set_data(xy_data)

			self.xy_canvas.restore_region(self.xy_background)
			self.xy_axis.draw_artist(self.xy_line)
			self.xy_canvas.blit(self.xy_axis.bbox)
			self.xz_canvas.restore_region(self.xz_background)
			self.xz_axis.draw_artist(self.xz_line)
			self.xz_canvas.blit(self.xz_axis.bbox)

		elif event.inaxes is self.wppsi_axis:
			wp, psi = event.xdata, event.ydata
			wppsi_data = self.wppsi_line.get_data()
			wppsi_data[1][self._ind] = psi
			self.wppsi_line.set_data(wppsi_data)
			self.wppsi_axis.draw_artist(self.wppsi_line)
			self.wppsi_canvas.blit(self.wppsi_axis.bbox)
			self.wppsi_canvas.restore_region(self.wppsi_background)

	def new_waypoint(self, event):
		if event.inaxes is self.xy_axis: y = event.ydata; z = 0
		elif event.inaxes is self.xz_axis: y = 0; z = event.ydata
		x = event.xdata

		xy_line_data = np.asarray(self.xy_line.get_data()).T
		xy_line_data = np.vstack((xy_line_data, [x, y])).T
		self.xy_line.set_data(xy_line_data)
		self.xy_canvas.restore_region(self.xy_background)
		self.xy_axis.draw_artist(self.xy_line)
		self.xy_canvas.blit(self.xy_axis.bbox)

		xz_line_data = np.asarray(self.xz_line.get_data()).T
		xz_line_data = np.vstack((xz_line_data, [x, z])).T
		self.xz_line.set_data(xz_line_data)
		self.xz_canvas.restore_region(self.xz_background)
		self.xz_axis.draw_artist(self.xz_line)
		self.xz_canvas.blit(self.xz_axis.bbox)

		wppsi_line_data = np.asarray(self.wppsi_line.get_data()).T
		wppsi_line_data = np.vstack((wppsi_line_data, [len(wppsi_line_data), 0])).T
		self.wppsi_line.set_data(wppsi_line_data)
		self.wppsi_canvas.restore_region(self.wppsi_background)
		self.wppsi_axis.draw_artist(self.wppsi_line)
		self.wppsi_canvas.blit(self.wppsi_axis.bbox)

	def get_data(self):
		# Position data
		xy = self.xy_line.get_data()
		xz = self.xz_line.get_data()
		wppsi = self.wppsi_line.get_data()
		x = [[i] for i in xy[0]]
		y = [[i] for i in xy[1]]
		z = [[i] for i in xz[1]]
		psi = [[i] for i in wppsi[1]]

		# Begin with no speed or acceleration
		x[0].extend([0.0, 0.0])
		y[0].extend([0.0, 0.0])
		z[0].extend([0.0, 0.0])
		psi[0].extend([0.0, 0.0])
		# End with no speed or acceleration
		x[-1].extend([0.0, 0.0])
		y[-1].extend([0.0, 0.0])
		z[-1].extend([0.0, 0.0])
		psi[-1].extend([0.0, 0.0])

		return np.array(x), np.array(y), np.array(z), np.array(psi)

if __name__ == '__main__':
	editor = WindowManager()

