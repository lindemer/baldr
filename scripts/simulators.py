'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

from scipy.integrate import ode
import models, trajectories, controllers
import numpy as np

class Simulation():
	def __init__(self, q, t1=10, dt=0.01, tscale=1.0):
		# queue for updating the gui progress bar
		self.post = q

		# time settings
		self.t0 = 0			# initial time
		self.t1 = t1			# final time
		self.dt = dt			# time step for integration
		self.tscale = tscale		# time scale for blender render

		# initial values
		x0     = 0.0;	dx0     = 0.0
		y0     = 0.0;	dy0     = 0.0
		z0     = 0.0;	dz0     = 0.0
		phi0   = 0.0;	dphi0   = 0.0
		theta0 = 0.0;	dtheta0 = 0.0
		psi0   = 0.0;	dpsi0   = 0.0
		self.init_state = np.array([	x0,		dx0, 
						y0,		dy0, 
						z0,		dz0,
						phi0,		dphi0,
						theta0,		dtheta0,
						phi0,		dphi0   	])

		self.data = dict(	x=[], 		y=[], 		z=[], 
					xr=[], 		yr=[], 		zr=[],		psir=[],
					phi=[], 	theta=[], 	psi=[],		t=[],
					u1=[], 		u2=[], 		u3=[], 		u4=[],
					u1r=[], 	u2r=[], 	u3r=[], 	u4r=[],
					omega1=[], 	omega2=[], 	omega3=[], 	omega4=[]	)

	def set_controller(self, name):
		if name == 'flat control law':		self.controller = controllers.FlatControlLaw()
		elif name == 'cascade interactive':	self.controller = controllers.CascadeInteractive()
		elif name == 'cascade tracking':	self.controller = controllers.CascadeTracking()
		elif name == 'discrete pid':		self.controller = controllers.DiscretePid()

	def set_model(self, name):
		if name == 'linear':		self.model = models.Linear()
		elif name == 'non-linear 1':	self.model = models.NonLinear1()
		elif name == 'non-linear 2':	self.model = models.NonLinear2()
		elif name == 'non-linear 3':	self.model = models.NonLinear3()

	def set_trajectory(self, name, filename=None):
		if name == 'spiral function':		self.trajectory = trajectories.Spiral()
		elif name == 'tangential function':	self.trajectory = trajectories.Tangential()
		elif name == 'custom':
			import pickle
			datafile = open(filename, 'rb')
			custom = pickle.load(datafile)
			self.trajectory = custom
			self.t1 = custom.k
			datafile.close()

	def set_integrator(self, name, **kwargs):
		self.f = ode(self.model.integration_loop)
		self.f.set_integrator(name, **kwargs)
		self.f.set_initial_value(self.init_state, self.t0)

	def process_data(self):
		while self.f.successful() and self.f.t < self.t1:
			r = self.trajectory(self.f.t)
			u, ur = self.controller(self.f.y, r)
			self.model.update(u)
			self.f.integrate(self.f.t + self.dt)
			omega = self.model.get_omega()
			self.log_data(self.f.t, u, ur, self.f.y, r, omega)
			self.post.put(self.f.t)

	def log_data(self, t, u, ur, y, r, omega):
		self.data['t'].append(t)
		self.data['u1'].append(u[0])
		self.data['u2'].append(u[1])
		self.data['u3'].append(u[2])
		self.data['u4'].append(u[3])
		self.data['u1r'].append(ur[0])
		self.data['u2r'].append(ur[1])
		self.data['u3r'].append(ur[2])
		self.data['u4r'].append(ur[3])
		self.data['x'].append(y[0])
		self.data['y'].append(y[2])
		self.data['z'].append(y[4])
		self.data['phi'].append(y[6])
		self.data['theta'].append(y[8])
		self.data['psi'].append(y[10])
		self.data['xr'].append(r[0][0])
		self.data['yr'].append(r[1][0])
		self.data['zr'].append(r[2][0])
		self.data['psir'].append(r[3][0])
		self.data['omega1'].append(omega[0])
		self.data['omega2'].append(omega[1])
		self.data['omega3'].append(omega[2])
		self.data['omega4'].append(omega[3])

class MatplotlibAnalysis(Simulation):
	def run(self):
		import flightplot
		plot = flightplot.FlightPlot(self.data)

class MorseTracking(Simulation):
	def run(self):
		import sys, time
		try: from pymorse import Morse
		except ImportError:
			print('Interfacing with MORSE requires Python 3.X and the pymorse module!')
			sys.exit(1)
		with Morse() as simu:
			for i in range(len(self.data['t'])):
				transporter = simu.qr.transporter
				transporter.publish({	'x':     self.data['x'][i],
							'y':     self.data['y'][i], 
							'z':     self.data['z'][i], 
							'phi':   self.data['phi'][i], 
							'theta': self.data['theta'][i], 
							'psi':   self.data['psi'][i]})
				time.sleep(self.dt / self.tscale)

class MorseInteractive(Simulation):
	def run(self):
		import sys, time
		try: from pymorse import Morse
		except ImportError:
			print('Interfacing with MORSE requires Python 3.X and the pymorse module!')
			sys.exit(1)
		with Morse() as simu:
			transporter = simu.qr.transporter
			keysensor = simu.qr.keysensor
			while self.f.successful():
				keys = eval(str(keysensor.get_local_data()))
				u = self.controller(self.f.y, keys)
				self.model.update(u)
				self.f.integrate(self.f.t + self.dt)
				transporter.publish({	'x':     self.f.y[0],
							'y':     self.f.y[2], 
							'z':     self.f.y[4], 
							'phi':   self.f.y[6], 
							'theta': self.f.y[8], 
							'psi':   self.f.y[10]})
				time.sleep(self.dt / self.tscale)

