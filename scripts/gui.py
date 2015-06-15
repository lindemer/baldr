'''
	Developed by Samuel Tanner Lindemer 2015
	Le Laboaratoire des Signaux et Systemes
	Centrale-Supelec Universite, Gif-sur-Yvette, France
	samuel.lindemer@lss.supelec.fr
'''

import os, sys, time, pickle
import numpy as np
from tkinter import *
from tkinter import ttk, filedialog

class Baldr():
	def __init__(self, request_q, progress_q, simulation_q):
		self.request_q = request_q
		self.progress_q = progress_q
		self.simulation_q = simulation_q
		self.simulation_isrunning = False
		self.open_filename = None

	def tk_mainloop(self):
		self.root = Tk()
		self.root.title('baldr')
		content = ttk.Frame(self.root, padding=(10, 10))

		n = ttk.Notebook(content)
		self.f0 = ttk.Frame(n)
		self.f1 = ttk.Frame(n)
		self.f2 = ttk.Frame(n)
		n.add(self.f0, text='Simulators', padding=(5, 5))
		n.add(self.f1, text='Integrators', padding=(5, 5))
		n.add(self.f2, text='Trajectories', padding=(5, 5))
		n.pack(fill=BOTH, pady=5, expand=1)

		# Main GUI data structure.
		# The first two tabs (simulation settings and integration settings) require constant
		# widget toggling and string conversion. The functionality of each widget is stored
		# here. The third tab (trajectories) is much simpler.
		self.params = {}
		self.params['simulator'] =     {'name':						'Simulation Settings',
										'input_field': 				StringVar(),
										'input_options':       		{'matplotlib': 				{'initiate_on_selection':  	['dt', 'model', 'trajectory', 'controller (tracking)'],
																								 'clear_on_selection': 		['tscale', 't1', 'controller (interactive)']},
																	 'MORSE (tracking)':     				{'initiate_on_selection': 	['dt', 'tscale', 'model', 'trajectory', 'controller (tracking)'],
																								 'clear_on_selection': 		['t1', 'controller (interactive)']},
																	 'MORSE (interactive)':{'initiate_on_selection': 	['dt', 'tscale', 'model', 'controller (interactive)'],
										 														 'clear_on_selection': 		['t1', 'trajectory', 'controller (tracking)']}},
										'default_value': 	'',
										'value':		None,
										'section':		self.f0,
										'row':			0,
										'widgets':		[],
										'widget_type':	'radiobutton',
										'datatype_expected':'string'}
		self.params['t1'] =				{'name':		'duration [s]',
										'input_field':	StringVar(),
										'default_value':'10',
										'value':		None,
										'section':		self.f0,
										'row':			5,
										'widgets':		[],
										'widget_type':	'entry',
										'datatype_expected':'float'}
		self.params['dt'] =				{'name':	'integration time step [s]',
										'input_field':		StringVar(),
										'default_value':	'0.01',
										'value':		None,
										'section':		self.f0,
										'row':			6,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['tscale'] =			{'name':			'blender render speed',
										'input_field':		StringVar(),
										'default_value':	'1.0',
										'value':		None,
										'section':		self.f0,
										'row':			7,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['controller (interactive)'] = 	{'name':	'controller (interactive)',
										'input_field': 		StringVar(),
										'input_options':       {'cascade':  	{'initiate_on_selection': 	[],
															'clear_on_selection': 		[]},
													'discrete pid':	{'initiate_on_selection': 	[],
															'clear_on_selection': 		[]}},
										'default_value': 	'cascade',
										'value':		None,
										'section':		self.f0,
										'row':			8,
										'widgets':		[],
										'widget_type':		'combobox',
										'datatype_expected':	'string'}
		self.params['controller (tracking)'] =		{'name':	'controller (tracking)',
										'input_field': 		StringVar(),
										'input_options':       {'cascade':  	{'initiate_on_selection': 	[],
															'clear_on_selection': 		[]},
													'flat control law':	{'initiate_on_selection': 	[],
															'clear_on_selection': 		[]}},
										'default_value': 	'cascade',
										'value':		None,
										'section':		self.f0,
										'row':			8,
										'widgets':		[],
										'widget_type':		'combobox',
										'datatype_expected':	'string'}
		self.params['model'] = 	      	{'name':			'model',
										'input_field': 		StringVar(),
										'input_options':       {'linear':    		{'initiate_on_selection': 	[],
																					'clear_on_selection': 		[]},
																'non-linear 1':		{'initiate_on_selection': 	[],
																					'clear_on_selection': 		[]},
																'non-linear 2':		{'initiate_on_selection': 	[],
																					'clear_on_selection': 		[]},
																'non-linear 3':		{'initiate_on_selection': 	[],
																					'clear_on_selection': 		[]}},
										'default_value': 	'linear',
										'value':		None,
										'section':		self.f0,
										'row':			8,
										'widgets':		[],
										'widget_type':		'combobox',
										'datatype_expected':	'string'}
		self.params['trajectory'] =    {'name':			'trajectory',
										'input_field': 		StringVar(),
										'input_options':	{'custom':			{'initiate_on_selection': 	[],
																					'clear_on_selection': 		['t1']},
															'spiral function': 		{'initiate_on_selection': 	['t1'],
																					'clear_on_selection': 		[]},
															'tangential function':  {'initiate_on_selection': 	['t1'],
																					'clear_on_selection': 		[]}},
										'default_value': 	'custom',
										'value':		None,
										'section':		self.f0,
										'row':			9,
										'widgets':		[],
										'widget_type':		'combobox',
										'datatype_expected':	'string'}
		self.params['integrator'] =    {'name': 'Integration Settings',
										'input_field': StringVar(),
										'input_options': 	{'dop853': 	{'initiate_on_selection': 	['atol', 'rtol', 'nsteps', 'max_step'],
																		'clear_on_selection': ['min_step', 'order', 'max_order_ns', 'max_order_s']},
															'dopri5': 	{'initiate_on_selection': ['atol', 'rtol', 'nsteps', 'max_step'],
																		'clear_on_selection': ['min_step', 'order', 'max_order_ns', 'max_order_s']},
															'lsoda': 	{'initiate_on_selection': ['atol', 'rtol', 'nsteps', 'min_step', 'max_step', 'max_order_ns', 'max_order_s'],
																		'clear_on_selection': ['order']},
															'vode':		{'initiate_on_selection': ['atol', 'rtol', 'nsteps', 'min_step', 'max_step', 'order'],
																		'clear_on_selection': ['max_order_ns', 'max_order_s']},
															'zvode':    {'initiate_on_selection': 	['atol', 'rtol', 'nsteps', 'min_step', 'max_step','order'],
																		'clear_on_selection': 		['max_order_ns', 'max_order_s']}},
										'default_value': 	'dopri5',
										'value':		None,
										'section':		self.f1,
										'row':			0,
										'widgets':		[],
										'widget_type':		'combobox',
										'datatype_expected':	'string'}

		# all of the following keys' default_value fields were copied directly from the scipy.integrate module
		self.params['atol'] = 	      	{'name':			'atol',
										'input_field':		StringVar(),
										'default_value':	'1e-12',
										'value':		None,
										'section':		self.f1,
										'row':			6,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['rtol'] = 	   		{'name':			'rtol',
										'input_field':		StringVar(),
										'default_value':	'1e-6',
										'value':		None,
										'section':		self.f1,
										'row':			7,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['nsteps'] =    		{'name':			'nsteps',
										'input_field':		StringVar(),
										'default_value':	'500',
										'value':		None,
										'section':		self.f1,
										'row':			8,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'integer'}
		self.params['min_step'] =     	{'name':			'min_step',
										'input_field':		StringVar(),
										'default_value':	'0.0',
										'value':		None,
										'section':		self.f1,
										'row':			9,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['max_step'] = 		{'name':			'max_step',
										'input_field':		StringVar(),
										'default_value':	'0.0',
										'value':		None,
										'section':		self.f1,
										'row':			10,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'float'}
		self.params['order'] =			{'name':			'order',
										'input_field':		StringVar(),
										'default_value':	'12',
										'value':		None,
										'section':		self.f1,
										'row':			11,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'integer'}
		self.params['max_order_ns'] =	{'name':			'max_order_ns',
										'input_field':		StringVar(),
										'default_value':	'12',
										'value':		None,
										'section':		self.f1,
										'row':			12,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'integer'}
		self.params['max_order_s'] =	{'name':			'max_order_s',
										'input_field':		StringVar(),
										'default_value':	'5',
										'value':		None,
										'section':		self.f1,
										'row':			13,
										'widgets':		[],
										'widget_type':		'entry',
										'datatype_expected':	'integer'}

		# simulation progressbar
		self.progress_var = IntVar()
		self.simulation_progressbar = ttk.Progressbar(content, mode='determinate', variable=self.progress_var)
		self.simulation_progressbar.pack(fill=X, pady=5)

		# initialize button
		self.initialize_button = ttk.Button(content, text='initialize', command=(lambda *args: self.initialize_clicked()))
		self.initialize_button.pack(fill=BOTH, pady=5)
		self.disable(self.initialize_button)
		# startstop button
		self.startstop_button = ttk.Button(content, text='start', command=(lambda *args: self.startstop_clicked()))
		self.startstop_button.pack(fill=BOTH, pady=5)
		self.disable(self.startstop_button)

		# create and format the input widgets and labels for the first two tabs of the GUI, add them to the self.params dictionary
		for data_field in sorted(self.params):
			if self.params[data_field]['widget_type'] == 'radiobutton':
				self.params[data_field]['label'] = Label(self.params[data_field]['section'], text=self.params[data_field]['name'], padx=5, pady=5)
				self.params[data_field]['label'].pack(fill=X)
				i = 0
				for option in sorted(list(self.params[data_field]['input_options'].keys())):
					self.params[data_field]['widgets'].append(ttk.Radiobutton(self.params[data_field]['section'], text=option, variable=self.params[data_field]['input_field'],
						value=option, command=(lambda *args, data_field=data_field: self.change_state(data_field))))
					self.params[data_field]['widgets'][i].pack(fill=X, ipady=3)
					i += 1
		for data_field in sorted(self.params):
			if self.params[data_field]['widget_type'] == 'combobox':
				self.params[data_field]['label'] = Label(self.params[data_field]['section'], text=self.params[data_field]['name'], padx=5, pady=5)
				self.params[data_field]['label'].pack(fill=X)
				self.params[data_field]['widgets'].append(ttk.Combobox(self.params[data_field]['section'], textvariable=self.params[data_field]['input_field'], state='readonly'))
				self.params[data_field]['widgets'][0].pack(fill=X, pady=2)
				self.params[data_field]['widgets'][0]['values'] = sorted(list(self.params[data_field]['input_options'].keys()))
				self.params[data_field]['widgets'][0].bind('<<ComboboxSelected>>', (lambda *args, data_field=data_field: self.change_state(data_field)))
		for data_field in sorted(self.params):
			if self.params[data_field]['widget_type'] == 'entry':
				self.params[data_field]['label'] = Label(self.params[data_field]['section'], text=self.params[data_field]['name'], padx=5)
				self.params[data_field]['label'].pack(fill=X, pady=5)
				self.params[data_field]['widgets'].append(ttk.Entry(self.params[data_field]['section'], textvariable=self.params[data_field]['input_field']))
				self.params[data_field]['widgets'][0].pack(fill=X)
				self.disable(self.params[data_field]['widgets'][0])

		# trajectory tab widgets and formatting
		draw_title = Label(self.f2, text='Trajectory Drawing Tool', padx=5, pady=5)
		draw_title.pack(fill=X)

		self.power_var = StringVar()
		self.power_var.set('10.00')
		power_label = Label(self.f2, text='power\n(u1 limiter [N])', padx=5)
		power_label.pack(fill=X, pady=5)
		power_entry = ttk.Entry(self.f2, textvariable=self.power_var)
		power_entry.pack(fill=X)
		self.tilt_var = StringVar()
		self.tilt_var.set('0.25')
		tilt_label = Label(self.f2, text='tilt\n(theta & phi limiter [pi rad])', padx=5)
		tilt_label.pack(fill=X, pady=5)
		tilt_entry = ttk.Entry(self.f2, textvariable=self.tilt_var)
		tilt_entry.pack(fill=X)
		self.guess_var = StringVar()
		self.guess_var.set('5.00')
		guess_label = Label(self.f2, text='poly piece guess [s]', padx=5)
		guess_label.pack(fill=X, pady=5)
		guess_entry = ttk.Entry(self.f2, textvariable=self.guess_var)
		guess_entry.pack(fill=X)

		self.save_button = ttk.Button(self.f2, text='create new', command=(lambda *args: self.save_clicked()))
		self.save_button.pack(fill=BOTH, pady=5)
		self.open_button = ttk.Button(self.f2, text='open existing', command=(lambda *args: self.open_clicked()))
		self.open_button.pack(fill=BOTH, pady=5)

		# format the window for resizing
		self.root.columnconfigure(0, weight=1)
		self.root.rowconfigure(0, weight=1)
		content.grid(column=0, row=0, sticky=(N, S, E, W))
		content.columnconfigure(0, weight=1)
		content.columnconfigure(1, weight=2)
		content.columnconfigure(2, weight=2)
		content.columnconfigure(3, weight=1)

		for i in range(7):
			content.rowconfigure(i, weight=1)

		# set the minimum window size
		self.root.update()
		self.root.minsize(self.root.winfo_width(), self.root.winfo_height())
	
		# run self.window_closed upon window closure
		self.root.wm_protocol('WM_DELETE_WINDOW', self.window_closed)

		# start the main gui loop
		self.root.mainloop()

	def change_state(self, *args):
		if args: 
			data_field = args[0]
			for option in list(self.params[data_field]['input_options'].keys()):
				if self.params[data_field]['input_field'].get() == option:
					self.clear_widgets(self.params[data_field]['input_options'][option]['clear_on_selection'])
					self.initiate_widgets(self.params[data_field]['input_options'][option]['initiate_on_selection'])
		# if all entries are valid and there is not currently a simulation running
		if self.get_gui_state() and not self.simulation_isrunning:
			# if a simulator and an integrator have been selected
			if self.params['simulator']['value'] and self.params['integrator']['value']:
				# if bspline is the selcted trajectory, then a file must be loaded
				if self.params['trajectory']['value'] == 'custom' and self.open_filename:
					self.enable(self.initialize_button)
				elif self.params['trajectory']['value'] != 'custom':
					self.enable(self.initialize_button)
				else:
					self.disable(self.initialize_button)

	def start_progressbar(self):
		start_time = time.time()
		print('Loading...', end=' ', flush=True)
		progress = 0
		if self.params['trajectory']['value'] == 'custom':
			maximum = self.custom_tmax
		else:
			maximum = self.params['t1']['value']
		self.simulation_progressbar['maximum'] = maximum
		while progress < maximum:
			try: progress = float(self.progress_q.get(block=False))
			except: pass
			else: 
				self.progress_var.set(progress)
				self.simulation_progressbar.update()
		print('Done!')
		print('Computation time:', round(time.time() - start_time, 2), 's')
		

	def initialize_clicked(self):
		if self.get_gui_state():
			self.disable(self.initialize_button)
			self.disable(self.open_button)
			self.disable(self.save_button)
			# send spawn request to request_handler process
			settings = self.get_settings()
			self.request_q.put(settings)
			self.simulation_isrunning = True
			if self.params['simulator']['value'] != 'MORSE (interactive)': 
				self.start_progressbar()
			else: 
				print('Loading... Done!')
				print('\
Key bindings for MORSE interactive:\n\
WASD:           move camera\n\
ctrl + mouse:   change camera angle\n\
J, L:           roll\n\
K, I:           pitch\n\
arrow keys:     yaw, thrust')
				self.simulation_progressbar['maximum'] = 1
				self.progress_var.set(1)
			self.enable(self.startstop_button)

	def open_clicked(self):
		self.open_filename = filedialog.askopenfilename(initialdir=self.get_save_dir())
		if self.open_filename:
			self.change_state()
			datafile = open(self.open_filename, 'rb')
			custom = pickle.load(datafile)
			self.custom_tmax = custom.k
			datafile.close()
			print('Custom piecewise polynomial trajectory data file loaded.\nPath:', self.open_filename, '\nTrajectory length:', round(custom.k, 2), 's', end='\n\n')
	
	def save_clicked(self):
		request = {	'component': 'draw',
					'draw': {'args': [None],
					'kwargs': {} } }
		try:
			request['draw']['kwargs']['power'] = float(self.power_var.get())
			if self.tilt_var.get(): request['draw']['kwargs']['tilt'] = float(self.tilt_var.get()) * np.pi
			else: request['draw']['kwargs']['tilt'] = None
			request['draw']['kwargs']['guess'] = float(self.guess_var.get())
		except ValueError: pass
		else:
			request['draw']['args'][0] = filedialog.asksaveasfilename(initialdir=self.get_save_dir(), initialfile='piecewise_polynomial.dat')
			if request['draw']['args'][0]:
				print ('\
Running trajectory drawing tool in a new process. Key bindings:\n\
shift: run optimization routine\n\
Z: save trajectory to data file\n\
T: toggle waypoint adjustment\n\
Path:', request['draw']['args'][0], end='\n\n')
				self.request_q.put(request)
		
	def startstop_clicked(self):
		if self.startstop_button['text'] == 'start':
			print('Running simulation in a new process...', end=' ', flush=True)
			self.simulation_q.put('start')
			self.startstop_button['text'] = 'stop'
			self.simulation_isrunning = True
			self.disable(self.initialize_button)
		elif self.startstop_button['text'] == 'stop':
			self.request_q.put('terminate')
			self.startstop_button['text'] = 'start'
			self.simulation_isrunning = False
			self.progress_var.set(0)
			self.change_state()
			self.disable(self.startstop_button)
			self.enable(self.open_button)
			self.enable(self.save_button)
			print('Stopped.\nProcess terminated.', end='\n\n')

	def window_closed(self):
		# send exit request to request_handler process
		self.request_q.put('exit')
		sys.exit(1)

	def get_gui_state(self):
		try:
			# attempt to convert each input field to its expected data type
			for data_field in self.params:
				for widget in self.params[data_field]['widgets']:
					# if widget is enabled
					if self.get_state(widget):
						if self.params[data_field]['datatype_expected'] == 'string':
							# the default return type of StringVal().get() is string
							self.params[data_field]['value'] = self.params[data_field]['input_field'].get()
						elif self.params[data_field]['datatype_expected'] == 'integer':
							self.params[data_field]['value'] = int(self.params[data_field]['input_field'].get())
						elif self.params[data_field]['datatype_expected'] == 'float':
							self.params[data_field]['value'] = float(self.params[data_field]['input_field'].get())
					else: self.params[data_field]['value'] = None
			# conversion of all inputs was successful
			return True

		except ValueError:
			# invalid entry detected
			return False

	# these functions make tkinter's enable/disable methods more readable in python code
	def disable(self, widget):
		widget.state(['disabled'])
	def enable(self, widget):
		widget.state(['!disabled'])
	def get_state(self, widget):
		if widget.instate(['!disabled']): return True
		elif widget.instate(['disabled']): return False
	
	# turn widgets on and off
	def clear_widgets(self, names):
		for name in names:
			for widget in self.params[name]['widgets']:
				self.disable(widget)
			self.params[name]['input_field'].set('')
	def initiate_widgets(self, names):
		for name in names:
			for widget in self.params[name]['widgets']:
				self.enable(widget)
			self.params[name]['input_field'].set(self.params[name]['default_value'])

	def get_save_dir(self):
		# navigates up one directory, and then down into the 'save' directory
		cwd = os.path.dirname(os.path.abspath(__file__))
		save_dir = cwd.split(os.sep)
		save_dir = save_dir[:-1]
		save_dir.append('save')
		save_dir = os.sep.join(save_dir)
		return save_dir

	# create the dictionary of simulation settings to put on the queue
	def get_settings(self):
		simulator_kwargs = ['t1', 'dt', 'tscale']
		integrator_kwargs = ['atol', 'rtol', 'nsteps', 'min_step', 'max_step', 'order', 'max_order_ns', 'max_order_s']
		settings = {	'component': 'simulator',
				'simulator': {	'name': self.params['simulator']['value'],
				'kwargs': {}					},
				'model':     {	'name': self.params['model']['value']		},
				'trajectory':{	'name': self.params['trajectory']['value'],
				'kwargs': 	 {	'filename': self.open_filename}	},
				'integrator':{  'name': self.params['integrator']['value'],
				'kwargs': {}	}}
		if self.params['simulator']['value'] == 'MORSE (tracking)' or self.params['simulator']['value'] == 'matplotlib':
			settings['controller'] = {'name': self.params['controller (tracking)']['value'], 'kwargs': {}}
		elif self.params['simulator']['value'] == 'MORSE (interactive)':
			settings['controller'] = {'name': self.params['controller (interactive)']['value'], 'kwargs': {}}
		# only send the kwargs that are defined
		for data_field in self.params:
			if self.params[data_field]['value'] is not None:
				if data_field in simulator_kwargs:
					settings['simulator']['kwargs'][data_field] = self.params[data_field]['value']
				if data_field in integrator_kwargs:
					settings['integrator']['kwargs'][data_field] = self.params[data_field]['value']
		return settings

