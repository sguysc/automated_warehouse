#!/usr/bin/env python

import numpy as np
import json
#import threading
import multiprocessing
import os, sys
from time import localtime, strftime
import logging # can't handle the ros logging :(

#import main as mn
import Jackal as mn

from shapely.geometry import Polygon, box, Point

import Tkinter as tk
import tkMessageBox, tkFileDialog, Tkconstants

import ROSUtilities as RU
import warehouse_map as wm

# ROS stuff
import rospy

if os.name == 'posix' and sys.version_info[0] < 3:
    import subprocess32 as subprocess
else:
    import subprocess
	
CELL_SIZE = 0.4

# helper class to get callbacks from text 
class CustomText(tk.Text):
    def __init__(self, *args, **kwargs):
        """A text widget that report on internal widget commands"""
        tk.Text.__init__(self, *args, **kwargs)

        # create a proxy for the underlying widget
        self._orig = self._w + "_orig"
        self.tk.call("rename", self._w, self._orig)
        self.tk.createcommand(self._w, self._proxy)

    def _proxy(self, command, *args):
        cmd = (self._orig, command) + args
        result = self.tk.call(cmd)

        if command in ("insert", "delete", "replace"):
            self.event_generate("<<TextModified>>")

        return result

class GUI:
	def __init__(self):

		self.main = tk.Tk() 
		self.main.title('Automated Warehouse in a Day - GUI') 
		
		self.filename = ''
		self.file_loaded = False
		self.has_fixed_step_changed = None
		self.has_task_step_changed = None
		self.add_new_obs = False
		self.add_new_nez = False
		self.add_new_ow = False
		self.coord1 = None
		self.coord2 = None
		self.RobotObjects = {}

		# the map canvas
		self.canvas_height=600
		self.canvas_width=600

		mapframe = tk.LabelFrame(self.main, text="Map", width=self.canvas_width, height=self.canvas_height)
		mapframe.grid(row=0,column=0) 
		self.map = tk.Canvas(mapframe, width=self.canvas_width, height=self.canvas_height, bg='white') 
		self.map.bind("<Button-1>", self.get_lclicked_coord)
		self.map.bind("<Button-3>", self.get_rclicked_coord)
		#self.map.grid(row=0,column=0) 
		self.map.pack()

		self.map.create_line(0, 0, self.canvas_width, self.canvas_height ) 
		self.map.create_line(self.canvas_width, 0, 0, self.canvas_height ) 
		#self.map.create_polygon(0.,   0., 453., 600., fill='red', outline='black')
		
		# specification textbox
		specframe = tk.LabelFrame(self.main, text="Specification (task, fixed):", width=40, height=30)
		specframe.grid(row=0,column=1,sticky='NW') 
		
		self.edit_spec = CustomText(specframe, height=22, width=40) 
		self.edit_spec.bind("<<TextModified>>", self.check_changed_specification)
		self.edit_spec.insert(tk.END, 'task') 
		self.edit_spec.pack()
		
		self.edit_fixed_spec = CustomText(specframe, height=20, width=40)
		self.edit_fixed_spec.bind("<<TextModified>>", self.check_changed_fixed_specification)
		self.edit_fixed_spec.insert(tk.END, 'fixed') 
		self.edit_fixed_spec.pack()
		#self.has_fixed_step_changed = False
		
		# all sorts of buttons
		self.exit_button = tk.Button(self.main, text='Quit', fg ='red', width=25, command=self.check_before_quitting) 
		self.exit_button.grid(row=4,column=1) 
		
		frame = tk.Frame(self.main)
		frame.grid(row=2,column=0) 
		self.synth_button = tk.Button(frame, text='Synthesize', fg ='green', width=25, command=self.Synthesize) 
		#self.synth_button.grid(row=1,column=0) 
		self.synth_button.pack(side = tk.BOTTOM)
		self.robot_num = tk.Entry(frame)
		self.robot_num.insert(tk.END, '0')
		#self.robot_num.grid(row=1,column=0)
		self.robot_num.pack(side = tk.RIGHT)
		lbl = tk.Label(frame, text='Robot #') 
		lbl.pack(side = tk.RIGHT)
		
		frame2 = tk.Frame(self.main)
		frame2.grid(row=3,column=0) 
		self.exec_button = tk.Button(frame2, text='Execute', fg ='#1f0781', width=25, command=self.Execute) 
		self.exec_button.pack(side = tk.LEFT)
		#self.exec_button.grid(row=3,column=0) 
		self.stop_button = tk.Button(frame2, text='Stop', fg ='brown', width=25, command=self.StopExecution) 
		self.stop_button.pack(side = tk.RIGHT)
		#self.stop_button.grid(row=3,column=0) 
		
		frame3 = tk.Frame(self.main)
		frame3.grid(row=2,column=1) 
		self.add_obs_button = tk.Button(frame3, bitmap='gray75', width=15, command=self.add_obstacle_to_fixed) #text='add obs',
		self.add_obs_button.pack(side = tk.LEFT)
		self.add_nez_button = tk.Button(frame3, bitmap='error', width=15, command=self.add_nez_to_fixed) #text='add nez',
		self.add_nez_button.pack(side = tk.RIGHT)
		self.add_ow_button = tk.Button(frame3, bitmap='warning', width=15, command=self.add_ow_to_fixed) #text='add ow',
		self.add_ow_button.pack(side = tk.RIGHT)
		
		# handle the menu bar
		self.menu = tk.Menu(self.main) 
		self.main.config(menu=self.menu) 
		filemenu = tk.Menu(self.menu) 
		self.menu.add_cascade(label='File', menu=filemenu) 
		filemenu.add_command(label='Open...', command=self.OpenMapDialog) 
		filemenu.add_command(label='Save', command=self.SaveSpecification) 
		filemenu.add_separator() 
		filemenu.add_command(label='Exit', command=self.check_before_quitting) 
		helpmenu = tk.Menu(self.menu) 
		self.menu.add_cascade(label='Help', menu=helpmenu) 
		helpmenu.add_command(label='About', command=self.ShowAbout) 
	
		self.main.protocol("WM_DELETE_WINDOW", self.check_before_quitting)
		self.main.mainloop() 
	
	def check_before_quitting(self):
		if tkMessageBox.askokcancel("Quit", "Do you really wish to quit?"):
			self.main.destroy()
	
	def check_changed_specification(self, event):
		try:
			if(self.has_task_step_changed == False and (self.has_task_step_changed != None)):
				self.main.title('Automated Warehouse in a Day - GUI (%s*)' %self.filename.split('/')[-1])
		except:
			pass
		finally:
			self.has_task_step_changed = True
		
		return True

	def check_changed_fixed_specification(self, event):
		try:
			if(self.has_fixed_step_changed == False and (self.has_fixed_step_changed != None)):
				print('fixed part changed, going to synthesize from beginning')
				self.main.title('Automated Warehouse in a Day - GUI (%s*)' %self.filename.split('/')[-1])
		except:
			pass
		finally:
			self.has_fixed_step_changed = True
		
		return True
	
	def get_rclicked_coord(self, event):
		if(not self.file_loaded):
			return
		import pdb; pdb.set_trace()
		print "clicked at", event.x, event.y
			
	def get_lclicked_coord(self, event):
		if(not self.file_loaded):
			return
		if(self.add_new_obs == False and self.add_new_nez == False and self.add_new_ow == False):
			self.coord1 = None 
			self.coord2 = None
			return
		
		if(self.coord1 == None):
			self.coord1 = [event.y*self.pix2m+self.workspace.bounds[0], event.x*self.pix2m+self.workspace.bounds[1]]
		elif(self.coord2 == None):
			self.coord2 = [event.y*self.pix2m+self.workspace.bounds[0], event.x*self.pix2m+self.workspace.bounds[1]]
			#import pdb; pdb.set_trace()
			if(self.add_new_obs):
				try:
					i = 1
					# find the last one, and add a new obstacle
					while True:
						self.spec['obstacles']['obs%d' %i]
						i += 1
				except:
					self.spec['obstacles'].update({'obs%d' %i : self.coord1 + self.coord2})
			if(self.add_new_nez):
				try:
					i = 1
					while True:
						self.spec['no_entrance']['ne%d' %i]
						i += 1
				except:
					self.spec['no_entrance'].update({'ne%d' %i : self.coord1 + self.coord2})
			if(self.add_new_ow):
				# decide the direction of the arrow first
				dx = self.coord2[0] - self.coord1[0]
				dy = self.coord2[1] - self.coord1[1]
				direction = 'S'
				if(np.abs(dx) > np.abs(dy)):
					if(dx >= 0):
						direction = 'S'
					else:
						direction = 'N'
				else:
					if(dy >= 0):
						direction = 'E'
					else:
						direction = 'W'
					
				try:
					i = 1
					# find the last one, and add a new obstacle
					while True:
						self.spec['one_way']['%s_%d' %(direction, i)]
						i += 1
				except:
					self.spec['one_way'].update({'%s_%d' %(direction, i) : self.coord1 + self.coord2})
			self.LoadMap(reload=False) #because we did not save the spec file yet, it's just in self.spec
			self.DrawMap()
			self.PopulateTask()

			self.add_new_obs = False 
			self.add_new_nez = False
			self.add_new_ow  = False
			self.coord1 = None
			self.coord2 = None

	def ShowAbout(self):
		tkMessageBox.showinfo("About", "Warehouse Automation in a Day\n\nGuy Scher\nJune 2020")
		
	def OpenMapDialog(self, file=None):
		if(file is None):
			self.filename = tkFileDialog.askopenfilename(initialdir = ".",title = "Select file",filetypes = (("Spec. files","*.specification"),("all files","*.*")))
		# if the user did not cancel
		if(self.filename != () ):
			if(self.filename != '' ):
				print(self.filename)
				self.main.title('Automated Warehouse in a Day - GUI (%s)' %self.filename.split('/')[-1])
				self.has_fixed_step_changed = None
				self.has_task_step_changed = None
				
				self.LoadMap()
				self.LoadTask()
		
				self.DrawMap()
				self.PopulateTask()
		
				self.has_fixed_step_changed = False
				self.has_task_step_changed = False
			
				self.thread = {}

	def SaveSpecification(self):
		if(not self.file_loaded):
			return
		#import pdb; pdb.set_trace()
		spec_goals = self.edit_spec.get('1.0', tk.END)
		spec_fixed = self.edit_fixed_spec.get('1.0', tk.END)
		
		#new_spec = dict(workspace=self.spec['workspace'], obstacles=self.spec['obstacles'])
		new_spec = {}
					
		new_goals = json.loads(spec_goals)
		new_fixed = json.loads(spec_fixed)
		
		new_spec.update(new_goals)
		new_spec.update(new_fixed)
		new_spec.update({'active_robots': len(new_goals.items())})
		
		self.spec = new_spec.copy()
		
		# save to file (new synthesis loads from file, not directly from textbox)
		with open(self.filename, 'w') as outfile:
			json.dump(self.spec, outfile)
		print('Saved specification to file: %s' %(self.filename))
		
		self.OpenMapDialog(file=self.filename)
		
	def Synthesize(self):
		if(not self.file_loaded):
			return
		
		self.SynthesisProcedure()
		
	def StopExecution(self):
		if(not self.file_loaded):
			return
		
		robot_num = int(self.robot_num.get())
		#import pdb; pdb.set_trace()
		'''
		try:
			#does it exist yet?
			self.RobotObjects[robot_num].external_shutdown = True
		except:
			print('Robot%d does not seem to run' %robot_num)
		'''
		try:
			# or basically not run yet
			if(self.thread[robot_num].is_alive()):
				print('stopping robot #%d ...' %(robot_num))
				self.thread[robot_num].terminate()
		except:
			print('Robot%d does not seem to run' %robot_num)
		
	def Execute(self):
		if(not self.file_loaded):
			return
		
		robot_num = int(self.robot_num.get())
				
		try:
			# or basically not run yet
			if(self.thread[robot_num].is_alive()):
				print('stopping robot #%d ...' %(robot_num))
				self.thread[robot_num].terminate()
		except:
			pass
		finally:
			print('running robot #%d' %(robot_num))
			#self.thread.update({robot_num: threading.Thread(target=run_main_program, args=[robot_num, self.spec['active_robots'], 'Person']) })
			self.thread.update({robot_num: multiprocessing.Process(target=self.run_main_program, \
								args=(robot_num, self.spec, 'person_standing')) })
			'''
			total_robots = self.spec['active_robots']
			list_obs = 'person_standing'
			robot_goals = self.spec['robot%d'%robot_num]
			robot_init = robot_goals['goal1']
			#import pdb; pdb.set_trace()
			subprocess.Popen('%s/Jackal.py --i %d --n %d --obs %s --x0 %f --y0 %f --teta0=%f' \
						 %(os.getcwd(), robot_num, total_robots, list_obs, robot_init[0], robot_init[1], robot_init[2]))
			'''
		self.thread[robot_num].start()

		
	def DrawMap(self):
		self.map.delete('all')
		#
				
		dx = -self.workspace.bounds[0]+0.0
		dy = -self.workspace.bounds[1]+0.0	
		dpos = np.array([dx, dy])
		
		xycoords = self.workspace.exterior.coords[:]+dpos
		xycoords = np.floor(xycoords / self.pix2m)
		xycoords = np.vstack([xycoords[:,1], xycoords[:,0]]).transpose() # swap x-y to match my axes
		xycoords = xycoords.tolist()
		
		# set up grid and axes
		self.checkered(self.map, int(xycoords[0][1]), int(xycoords[1][0]), 10)

		self.map.create_polygon(*xycoords, fill='', outline='black', width='4')
		
		#import pdb; pdb.set_trace()
		for i, obstacle in enumerate(self.obs):
			xycoords = obstacle.exterior.coords[:]+dpos
			xycoords = np.floor(xycoords / self.pix2m)
			xycoords = np.vstack([xycoords[:,1], xycoords[:,0]]).transpose() # swap x-y to match my axes
			xycoords = xycoords.tolist()
			self.map.create_polygon(*xycoords, fill='#e1dfdf', outline='black')
			self.map.create_text((xycoords[0][0]+xycoords[1][0])/2., (xycoords[0][1]+xycoords[2][1])/2., text='%s'%(self.obs_names[i]))
		
		for i, nez in enumerate(self.no_enter):
			xycoords = nez.exterior.coords[:]+dpos
			xycoords = np.floor(xycoords / self.pix2m)
			xycoords = np.vstack([xycoords[:,1], xycoords[:,0]]).transpose() # swap x-y to match my axes
			xycoords = xycoords.tolist()
			self.map.create_polygon(*xycoords, fill='#ffaeae', outline='black')
			self.map.create_text((xycoords[0][0]+xycoords[1][0])/2., (xycoords[0][1]+xycoords[2][1])/2., text='%s'%(self.no_enter_names[i]))
		
		# south 
		for ow in self.one_ways[0]:
			xycoords = ow.bounds
			x1 = np.floor((ow.bounds[0]+dx)/ self.pix2m)
			y1 = np.floor((ow.bounds[1]+dy)/ self.pix2m)
			x2 = np.floor((ow.bounds[2]+dx)/ self.pix2m)
			y2 = np.floor((ow.bounds[3]+dy)/ self.pix2m)
			self.map.create_line((y1+y2)/2, x1, (y1+y2)/2, x2, arrow=tk.LAST, fill='#738744')
		# north
		for ow in self.one_ways[2]:
			xycoords = ow.bounds
			x1 = np.floor((ow.bounds[0]+dx)/ self.pix2m)
			y1 = np.floor((ow.bounds[1]+dy)/ self.pix2m)
			x2 = np.floor((ow.bounds[2]+dx)/ self.pix2m)
			y2 = np.floor((ow.bounds[3]+dy)/ self.pix2m)
			self.map.create_line((y1+y2)/2, x2, (y1+y2)/2, x1, arrow=tk.LAST, fill='#738744')
	
		# east 
		for ow in self.one_ways[1]:
			xycoords = ow.bounds
			x1 = np.floor((ow.bounds[0]+dx)/ self.pix2m)
			y1 = np.floor((ow.bounds[1]+dy)/ self.pix2m)
			x2 = np.floor((ow.bounds[2]+dx)/ self.pix2m)
			y2 = np.floor((ow.bounds[3]+dy)/ self.pix2m)
			self.map.create_line(y1, (x1+x2)/2, y2, (x1+x2)/2, arrow=tk.LAST, fill='#738744')
		# west
		for ow in self.one_ways[3]:
			xycoords = ow.bounds
			x1 = np.floor((ow.bounds[0]+dx)/ self.pix2m)
			y1 = np.floor((ow.bounds[1]+dy)/ self.pix2m)
			x2 = np.floor((ow.bounds[2]+dx)/ self.pix2m)
			y2 = np.floor((ow.bounds[3]+dy)/ self.pix2m)
			self.map.create_line(y2, (x1+x2)/2, y1, (x1+x2)/2, arrow=tk.LAST, fill='#738744')
		#import pdb; pdb.set_trace()
		for R, goals in enumerate(self.all_goals):
			for i, goal in enumerate(goals):
				center = [goal[0], goal[1]]
				x0 = (center[0]+dx-CELL_SIZE/2.)/ self.pix2m
				y0 = (center[1]+dy-CELL_SIZE/2.)/ self.pix2m
				x1 = (center[0]+dx+CELL_SIZE/2.)/ self.pix2m
				y1 = (center[1]+dy+CELL_SIZE/2.)/ self.pix2m
				self.map.create_oval(y0, x0, y1, x1, fill='#ffda9b', activefill='#ffaa9b')
				self.map.create_text((center[1]+dy)/self.pix2m, (center[0]+dx)/self.pix2m, text='R%dg%d'%(R,i))
		
		self.map.pack()
			
	def PopulateTask(self):
		self.edit_spec.delete('1.0', tk.END)
		self.edit_fixed_spec.delete('1.0', tk.END)
		
		#self.edit_spec.insert(tk.END, 'Goals: [x,y,theta]\n==================\n')
		self.edit_spec.insert(tk.END, '{\n')
		for R in range(self.spec['active_robots']):
			robot_goals = json.dumps(self.spec['robot%d' %R])
			self.edit_spec.insert(tk.END, '\"robot%d\":\n' %R)
			self.edit_spec.insert(tk.END, robot_goals)
			self.edit_spec.insert(tk.END, ',\n')
		tmp_str = self.edit_spec.get("1.0", tk.END)
		self.edit_spec.delete("1.0", tk.END)
		self.edit_spec.insert(tk.END, tmp_str[:-3])
		
		#import pdb; pdb.set_trace()
		self.edit_spec.insert(tk.END, '\n}\n')
			
		self.edit_fixed_spec.insert(tk.END, '{\n\"workspace\":\n')
		self.edit_fixed_spec.insert(tk.END, json.dumps(self.spec['workspace']))
		
		self.edit_fixed_spec.insert(tk.END, ',\n\"obstacles\":\n')
		self.edit_fixed_spec.insert(tk.END, json.dumps(self.spec['obstacles']))
		
		self.edit_fixed_spec.insert(tk.END, ',\n\"one_way\":\n')
		self.edit_fixed_spec.insert(tk.END, json.dumps(self.spec['one_way']))
		
		self.edit_fixed_spec.insert(tk.END, ',\n\"no_entrance\":\n')
		self.edit_fixed_spec.insert(tk.END, json.dumps(self.spec['no_entrance']))
		
		self.edit_fixed_spec.insert(tk.END, '}\n')
			
	
	def LoadTask(self):
		'''
		try:
			with open(self.filename, 'r') as spec_file:
				self.spec = json.load(spec_file)
		except:
			self.file_loaded = False
			print('Specification %s file has syntax error.' %(self.filename))
			raise
		'''
		self.file_loaded = True
		self.all_goals = []
		self.num_robots = self.spec['active_robots']
		for i in range(self.num_robots):
			robot     = self.spec['robot%d' %i]
			num_goals = robot['goals']
			goals = []
			for j in range(num_goals):
				goal = robot['goal%d' %(j+1)]
				goals.append(goal)
			self.all_goals.append(goals)

	
	def LoadMap(self, reload=True):
		try:
			if(reload):
				with open(self.filename, 'r') as spec_file:
					self.spec = json.load(spec_file)
		except:
			print('Specification %s file has syntax error.' %(self.filename))
			raise

		#import pdb; pdb.set_trace()
		self.workspace = box(*self.spec['workspace'])
		obstacles = self.spec["obstacles"]
		no_entrance = self.spec["no_entrance"]
		one_way = self.spec["one_way"]
		self.obs = []
		self.obs_names = []
		self.no_enter = []
		self.no_enter_names = []
		self.one_ways = [[], [], [], [] ]

		for key, val in obstacles.items():
			if(len(val)>0):
				self.obs.append(box(*val))
				self.obs_names.append(key)
		for key, val in no_entrance.items():
			if(len(val)>0):
				self.no_enter.append(box(*val))
				self.no_enter_names.append(key)
		for key, val in one_way.items():
			if(len(val)>0):
				if(key[0] == 'N'):
					numeric_key = 2
				elif(key[0] == 'E'):
					numeric_key = 1
				elif(key[0] == 'W'):
					numeric_key = 3
				else: 
					#key[0] == 'S'
					numeric_key = 0

				self.one_ways[numeric_key].append(box(*val))

		self.W_Height = self.workspace.bounds[2] - self.workspace.bounds[0]  # [m]
		self.W_Width  = self.workspace.bounds[3] - self.workspace.bounds[1] # [m]

		self.pix2m  = np.max([self.W_Width/self.canvas_width, self.W_Height/self.canvas_height])*1.0
		
		#import pdb; pdb.set_trace()

	# GUY: need to check this pipeline
	def SynthesisProcedure(self):	
		cell = CELL_SIZE
		
		if(self.has_fixed_step_changed == False):
			force = False
		else:
			force = True

		folders = self.filename.split('/')
		fname, ext = folders[-1].split('.')
		map_kind = fname
		#import pdb; pdb.set_trace()
		
		MP = wm.LoadMP(fName='MPLibrary.lib')
		wm.MotionPrimitivesToFile(map_kind, MP) # for loading in matlab
		# we load it again to get it from the file, in case user did not hit save
		workspace, obs, no_enter, one_ways = wm.ReplicateMap(map_kind=map_kind) #
		goals, robots_num = wm.GetGoals(map_kind=map_kind)
		# save it to file
		wm.MapToFile(map_kind.lower(), workspace, obs, goals, no_enter, one_ways, 1.0) # for loading in matlab

		DiGraph, ax = wm.PopulateMapWithMP(MP, workspace, obs, no_enter, one_ways, map_kind, cell_h=cell, cell_w=cell, force=force, plot=False)

		print('Searching for naive paths (disregarding no-entry zones) using graph search ...')
		for i in range(robots_num):
			# plot each individual robot's path
			path = wm.FindPathBetweenGoals(DiGraph, np.array(goals[i]) )
		
		# create the amount of jackals you want
		robots_ic = []
		for i in range(robots_num):
			robots_ic.append([goals[i][0][0], goals[i][0][1], goals[i][0][2]])
		RU.CreateJackals(map_kind, robots_ic)
		#
		if(self.has_fixed_step_changed):
			map_label_2_bit = wm.CreateSlugsInputFile(DiGraph, goals, MP, no_enter, robots_num, filename=map_kind) #'map_funnel')
		else:
			#import pdb; pdb.set_trace()
			robot_num = int(self.robot_num.get())
			map_label_2_bit = wm.UpdateGoalsSlugsInputFile(goals, robot_num, filename=map_kind)
		# the reverse dictionary is useful
		map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())

		if(self.has_task_step_changed):
			wm.CheckRealizeability(robots_num, filename=map_kind, robot_num=robot_num)
		else:
			wm.CheckRealizeability(robots_num, filename=map_kind)
			
		print('Done.')

	def add_obstacle_to_fixed(self):
		if(not self.file_loaded):
			return
		self.add_new_obs = not self.add_new_obs
	
	def add_nez_to_fixed(self):
		if(not self.file_loaded):
			return
		self.add_new_nez = not self.add_new_nez
	
	def add_ow_to_fixed(self):
		if(not self.file_loaded):
			return
		self.add_new_ow = not self.add_new_ow
	
	def checkered(self, canvas, canvas_height, canvas_width, line_distance):
		# vertical lines at an interval of "line_distance" pixel
		for x in range(line_distance,canvas_width,line_distance):
			canvas.create_line(x, 0, x, canvas_height, fill="#ececec")
			if((x-10)%50 == 0):
				canvas.create_line(x, canvas_height-10, x, canvas_height+10, fill="#000000")
				canvas.create_text(x, canvas_height+20, text='%.2f'%(x*self.pix2m+self.workspace.bounds[1]))
		# horizontal lines at an interval of "line_distance" pixel
		for y in range(line_distance,canvas_height,line_distance):
			canvas.create_line(0, y, canvas_width, y, fill="#ececec")
			if((y-10)%50 == 0):
				canvas.create_line(-10, y, 10, y, fill="#000000")
				canvas.create_text(30, y, text='%.2f'%(y*self.pix2m+self.workspace.bounds[0]))

	def run_main_program(self, robot_num, spec, obstacles):
		# option a
		'''
		os.system("./main.py --i %d --n %d --obs %s" %(robot_num, total_robots, obstacles))
		'''
		# option b
		#'''
		print('Controller for Jackal%d' %robot_num)
		robot_goals = spec['robot%d'%robot_num]
		total_robots = spec['active_robots']
		list_obs = obstacles.split(',')

		rospy.init_node('run_jackal_%d' %robot_num)#, log_level=rospy.DEBUG)
		self.RobotObjects.update({robot_num: mn.Jackal(robot_num, total_robots, list_obs, first_goal_for_gazebo=robot_goals['goal1'])})

		try:
			#rospy.spin()
			self.RobotObjects[robot_num].Run()
		except KeyboardInterrupt:
			pass
		except rospy.ROSInterruptException:
			pass
		except:
			print "Unexpected error:", sys.exc_info()[0]
			raise
		finally:
			timenow     = localtime()
			print('Ended program at ' + strftime('%H:%M:%S', timenow))
			self.RobotObjects[robot_num].Shutdown() #hopefully destroy the memory taken by slugs
		#'''
		# option c
		'''
		total_robots = spec['active_robots']
		robot_goals = spec['robot%d'%robot_num]
		robot_init = robot_goals['goal1']
		#self.RobotObjects.update({robot_num: subprocess.Popen('./Jackal.py --i %d --n %d --obs %s --x0 %f --y0 %f --teta0=%f' \
		#										   %(robot_num, total_robots, list_obs, robot_goals[0], robot_goals[1], robot_goals[2]) ) } ) #
		print('%s/Jackal.py --i %d --n %d --obs %s --x0 %f --y0 %f --teta0 %f' \
						 %(os.getcwd(), robot_num, total_robots, obstacles, robot_init[0], robot_init[1], robot_init[2]))
		#subprocess.Popen('./Jackal.py --i %d --n %d --obs %s --x0 %f --y0 %f --teta0 %f' \
		#				 %(robot_num, total_robots, obstacles, robot_init[0], robot_init[1], robot_init[2]))
		subprocess.Popen(['./Jackal.py', '--i', str(robot_num), '--n', str(total_robots), '--obs', obstacles, \
						  '--x0', str(robot_init[0]), '--y0', str(robot_init[1]), '--teta0', str(robot_init[2])], shell=True)
		'''
		# END ALL

	
if __name__ == '__main__':
	g = GUI()

	
