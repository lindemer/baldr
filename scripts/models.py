'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import numpy as np

class Model():
	def __init__(self):
		self.m	 	= 0.53					# total mass [kg]
		self.g 		= 9.81					# acceleration of gravity [m/s^2]
		self.Ix 	= 6.228e-3				# longitudinal inertia [kgm^2]
		self.Iy 	= 6.228e-3				# lateral inertia [kgm^2]
		self.Iz		= 1.121e-3				# vertial inertia [kgm^2]
		self.Jr  	= 6e-5					# rotor inertia [kgm^2]
		self.l   	= 0.232					# blade length [m]
		self.rho 	= 1.293					# air density [kgm^-3]
		self.R   	= 0.15					# rotor radius [m]
		self.A   	= np.pi * self.R**2			# rotor area [m^2]
		self.CT  	= 0.0158				# thrust coefficient
		self.CQ  	= 2.2e-3		
		self.c  	= self.CT * self.rho * self.A * self.R**2

	def update(self, u):
		self.u = u

	def compute_omega(self):
		''' 
		The four rotor speeds omega1, omega2, omega3 and omega4 are 
		computed from the four input commands u1, u2, u3, and u4 
		(thrust, roll, pitch, yaw). We do this by solving the 
		following system of equations:
		    u1 = T1 + T2 + T3 + T4
		    u2 = l(T4 - T2)
		    u3 = l(T3 - T1)
		    u4 = Q1 - Q2 + Q3 -Q4
		    Ti = Ct * rho * A * R^2 * omegai^2
		    Qi = Cq * rho * A * R^3 * omegai^2
		    omegar = omega1 - omega2 + omega3 - omega4 
		'''
		p =  self.u[2] / self.l
		q =  self.u[1] / self.l
		r =  self.u[0]
		s = (self.u[3] * self.CT) / (self.CQ * self.R)
		T1 = -0.5 * p +           0.25 * r + 0.25 * s
		T2 =          - 0.5 * q + 0.25 * r - 0.25 * s
		T3 =  0.5 * p +           0.25 * r + 0.25 * s
		T4 =            0.5 * q + 0.25 * r - 0.25 * s
		self.omega = np.array([	np.sqrt(np.abs(T1) / self.c),
					np.sqrt(np.abs(T2) / self.c),
					np.sqrt(np.abs(T3) / self.c),
					np.sqrt(np.abs(T4) / self.c)	])

	def get_omega(self):
		return self.omega

class Linear(Model):
	def integration_loop(self, t, X):
		self.compute_omega()
		x_        = X[1]
		y_        = X[3]
		z_        = X[5]
		phi_      = X[7]
		theta_    = X[9]
		psi_      = X[11]
		dx_       =  self.g * X[8]
		dy_       = -self.g * X[6]
		dz_       = (-self.m * self.g + self.u[0]) / self.m
		dphi_     = self.u[1] / self.Ix
		dtheta_   = self.u[2] / self.Iy
		dpsi_     = self.u[3] / self.Iz
		return np.array([	x_,   dx_,   y_,     dy_,     z_,   dz_,
					phi_, dphi_, theta_, dtheta_, psi_, dpsi_	])

class NonLinear1(Model):
	def integration_loop(self, t, X):
		self.compute_omega()
		x_	= X[1]
		y_	= X[3]
		z_	= X[5]
		phi_	= X[7]
		theta_	= X[9]
		psi_	= X[11]
		dx_ 	=   X[8] * self.u[0]  / self.m
		dy_	= (-X[6] * self.u[0]) / self.m
		dz_	= (-self.m * self.g + self.u[0]) / self.m
		dphi_	= self.u[1] / self.Ix
		dtheta_	= self.u[2] / self.Iy
		dpsi_	= self.u[3] / self.Iz
		return np.array([	x_,   dx_,   y_,     dy_,     z_,   dz_,
					phi_, dphi_, theta_, dtheta_, psi_, dpsi_	])

class NonLinear2(Model):
	def integration_loop(self, t, X):
		self.compute_omega()
		x_	= X[1]
		y_	= X[3]
		z_	= X[5]
		phi_ 	= X[7]
		theta_	= X[9]
		psi_	= X[11]
		dx_	= ( np.cos(X[6]) * np.sin(X[8])) * self.u[0] / self.m
		dy_	= (-np.sin(X[6]) * np.cos(X[8])) * self.u[0] / self.m
		dz_	= ( np.cos(X[6]) * np.cos(X[8])  * self.u[0] - self.m * self.g) / self.m
		dphi_	= (X[9] * X[11] * (self.Iy - self.Iz) + self.u[1]) / self.Ix
		dtheta_	= (X[7] * X[11] * (self.Iz - self.Ix) + self.u[2]) / self.Iy
		dpsi_	= (X[7] * X[9]  * (self.Ix - self.Iy) + self.u[3]) / self.Iz		
		return np.array([	x_,   dx_,   y_,     dy_,     z_,   dz_,
					phi_, dphi_, theta_, dtheta_, psi_, dpsi_	])

class NonLinear3(Model): 
	def integration_loop(self, t, X):
		self.compute_omega()
		omegaR = self.omega[0] - self.omega[1] + self.omega[2] - self.omega[3]
		x_	= X[1]
		y_	= X[3]
		z_	= X[5]
		phi_	= X[7]
		theta_	= X[9]
		psi_	= X[11]
		dx_	= ( np.cos(X[6]) * np.sin(X[8])) * self.u[0] / self.m
		dy_	= (-np.sin(X[6]) * np.cos(X[8])) * self.u[0] / self.m
		dz_	= ( np.cos(X[6]) * np.cos(X[8])  * self.u[0] - self.m * self.g) / self.m
		dphi_	= (X[9] * X[11] * (self.Iy - self.Iz) + self.u[1] - self.Jr * X[9] * omegaR) / self.Ix
		dtheta_	= (X[7] * X[11] * (self.Iz - self.Ix) + self.u[2] - self.Jr * X[7] * omegaR) / self.Iy 
		dpsi_	= (X[7] * X[9]  * (self.Ix - self.Iy) + self.u[3]) / self.Iz		
		return np.array([	x_,   dx_,   y_,     dy_,     z_,   dz_,
					phi_, dphi_, theta_, dtheta_, psi_, dpsi_	])

