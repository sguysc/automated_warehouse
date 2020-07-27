#!/usr/bin/env python

# general stuff
import numpy as np
import subprocess, os, sys
import dill
import re

import global_parameters as glob_p

# helper class to get transitions from funnel to funnel using interactive mode
class SlugsInterface():
	#'/home/gs679/Tools/slugs/src/slugs', '/home/cornell/Tools/slugs_ltl_stack/src/slugs'
	def __init__(self, name, simulate=False, slugsLink = None):
		self.name = name # name of map file to synthesize
		self.realizeable = True
		self.enabled = False # is the class operational? did initialize and are realizeable
		self._current_state = '' # string that holds the state
		self._trans_state = ''   # string that holds the new state after transitioning
		self._current_goal = 0 
		self._inputs = {} # names and values of current input (states)
		self._outputs = {} # names and values of current output (action)
		self._Nin = 0   # total number of inputs (self robot + restrictions)
		self._Nout = 0  # number of command bits
		self._Nself = 0 # number of self robot bits
		self._Nsens = 0 # number of sensors bits
		self._NGoals = 0 # total number of goals
		self._simulate = simulate
		self._state_index = 0
		
		if(slugsLink == None):
			slugsLink = glob_p.slugsLink
		
		if(self._simulate == False):
			# open the process of slugs and keep it open to recieve commands
			self.slugsProcess = subprocess.Popen(slugsLink + ' --interactiveStrategy ' + self.name + '.slugsin', \
												 bufsize=1048000, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid, \
												 stdin=subprocess.PIPE, stdout=subprocess.PIPE, \
												 close_fds='posix' in sys.builtin_module_names)
			# wait for synthesis to be done
			stderr, stdout = "",""

			# wait till we find out if the file is realizeable or not
			stdout += self.slugsProcess.stdout.readline().strip() # only: oneStepRecovery:0
			#ii=1
			while not stderr.endswith("Execution\n"): 
				# option a:
				# retrieve outputs
				stderr += self.slugsProcess.stderr.readline().strip()
				# exit if synthesis is done and ready for execution
				if "error" in stderr.lower():
					self.realizeable = False
					break
				elif ("unrealizable" in stderr) and ("oneStepRecovery" in stdout):
					self.realizeable = False
					break
				elif ("realizable" in stderr) and ("oneStepRecovery" in stdout):
					break
				#print('ii=%d' %(ii))
				#ii += 1
			'''
				# option b:
				stderr += self.slugsProcess.stderr.readline().strip()
			if( ("error" in stderr.lower() or "realizable" in stderr) and ("oneStepRecovery" in stdout)):
				self.realizeable = False
			'''
			#import pdb; pdb.set_trace()
			if(self.realizeable):		
				print('ready to rock & roll!')
				self.enabled = True
			else:
				#import pdb; pdb.set_trace()
				print('check your slugsin file, it is not realizeable ...')
		else:
			print('simulation is ready to rock & roll!')
			print('do not forget it\'s only simulated slugs')
			self.enabled = True
			self.realizeable = True
			

	# get the list of inputs
	def DiscoverInputs(self):
		if(self.enabled):
			if(self._simulate == True):
				self._Nsens = 9
				self._Nin = 20
				self._Nself = 11
				self._inputs = {'R1_8': 0, 'R1_7': 0, 'R1_2': 0, 'R1_3': 0, 'R1_0': 0, 'R1_1': 0, 'R1_6': 0,  \
							   'R@3': 0, 'R1_4': 0, 'R@10': 0, 'R@1': 0, 'R@0.0.1742': 0, 'R@2': 0, 'R@5': 0,\
							   'R@4': 0, 'R@7': 0, 'R@6': 0, 'R@9': 0, 'R@8': 0, 'R1_5': 0}
			else:
				self.slugsProcess.stdin.write("XPRINTINPUTS\n")
				self.slugsProcess.stdin.flush()

				stdout, stderr = "", ""
				# get the response
				while not stdout.endswith("\n\n"):
					stdout += self.slugsProcess.stdout.readline()

				inputs_state = stdout.replace(">","").strip()
				#print("inputs: {inputs_state}".format(inputs_state=inputs_state))
				list_of_inputs = inputs_state.split('\n')
				self._inputs = dict(zip(list_of_inputs, [0]*len(list_of_inputs)))
				self._Nin = len(list_of_inputs)
				self._Nself = 0
				# find out how many occupy the self robot. the idea is that it is represented
				# by R so all it's propositions are R@something
				for x in self._inputs.keys():
					if('R@' in x):
						self._Nself +=1
				self._Nsens = self._Nin - self._Nself
			
	def DiscoverOutputs(self):
		if(self.enabled):
			if(self._simulate == True):
				self._Nout = 4
				self._outputs = {'mp@3': 0, 'mp@2': 0, 'mp@1': 0, 'mp@0.0.9': 0}
			else:
				self.slugsProcess.stdin.write("XPRINTOUTPUTS\n")
				self.slugsProcess.stdin.flush()

				stdout, stderr = "", ""
				while not stdout.endswith("\n\n"):
				#while not ">" in stdout:
					stdout += self.slugsProcess.stdout.readline()

				outputs_state = stdout.replace(">","").strip()
				#print("outputs: {outputs_state}".format(outputs_state=outputs_state))
				list_of_outputs = outputs_state.split('\n')
				self._Nout = len(list_of_outputs)
				self._outputs = dict(zip(list_of_outputs, [0]*self._Nout))
			
	def DiscoverGoals(self):
		if(self.enabled):
			if(self._simulate == True):
				self._NGoals = 3
			else:
				self.slugsProcess.stdin.write("XGETNOFLIVENESSPROPERTIES\n")
				self.slugsProcess.stdin.flush()

				stdout, stderr = "", ""
				#while not stdout.endswith("\n"):
				#while not ">" in stdout:
				stdout += self.slugsProcess.stdout.readline()
				stdout += self.slugsProcess.stdout.readline()
				stdout += self.slugsProcess.stdout.readline()

				outputs_state = stdout.replace(">","").strip()
				#print("outputs: {outputs_state}".format(outputs_state=outputs_state))
				list_of_outputs = outputs_state.split('\n')
				self._NGoals = int(list_of_outputs[-1])
			
	# get the list of inputs, returns self_robot, restrictions
	def GetCurrentInputs(self):
		if(self.enabled):
			return self._current_state[0:self._Nself], self._current_state[self._Nself:]
		else:
			return '', ''

	# sets current position, if everything is as expected in start up or whenever you want to reset position
	# but don't know exactly which action you want to take
	def SetInitialPos(self, init_state, init_sense):
		if(self.enabled):
			if(self._simulate == True):
				#self._current_state = '111011011100000000000100'
				#self._current_state_n = [int(x) for x in list(self._current_state)]
				#self._current_goal = '0'
				#return self._current_state
				pass
			else:
				state_str = format(init_state, '0%db' %(self._Nself) )
				state_str = state_str[::-1]
				# convert dict of restrictions into a string
				sensed_inputs = ""
				for prop in range(self._Nsens):
					if prop in init_sense:
						sensed_inputs += "1" if (init_sense[prop] is True) else "0"
					else:
						sensed_inputs += "."
				state_str = state_str + sensed_inputs # restrictions
			
				state_str = state_str + '.'*self._Nout #do not disclose what action to take
				self.slugsProcess.stdin.write("XCOMPLETEINIT\n" + state_str)
				self.slugsProcess.stdin.flush()

				# iterate until we actually get our state
				#print('..')
				stdout, stderr = "", ""
				while not (re.search('[aAgGsS01]',stdout) or 'FORCEDNONWINNING' in stdout):
					stdout += self.slugsProcess.stdout.readline()

				print('resetted initial point')
				if 'FORCEDNONWINNING' in stdout:
					return ""
				
				init_state = stdout.replace(">","").strip()
				init_state = init_state.partition(",")[0]
				decipher_state = ''
				decipher_sense = {}
				action = ''
				# create list output with the current state prop assignments
        		# in the form of AaGgSs
        		# A: given true value,    a:given false value
        		# G: possible true value, g:possible false value
				for i in range(self._Nself):
					element = init_state[i]
					value = '1' if element == 'A' or element == 'G' or element == '1' or element == 'S' else '0'
					decipher_state = value + decipher_state # reverse the order of the variables
				#decipher_state = decipher_state[::-1] 
				for i in range(self._Nself, self._Nin):
					element = init_state[i]
					value = True if element == 'A' or element == 'G' or element == '1' or element == 'S' else False
					decipher_sense.update({(i-self._Nself):  value} )
				for i in range(self._Nin, self._Nin+self._Nout):
					element = init_state[i]
					value = '1' if element == 'A' or element == 'G' or element == '1' or element == 'S' else '0'
					action = value + action
				#action = action[::-1] # reverse the order of the variables
				#import pdb; pdb.set_trace()
				# now actually set this position
				self.SetCurrentPos(int(decipher_state, 2), decipher_sense, int(action,2))
				
				return self.GetNumericState()
		else:
			return ''
		
	# gets current position, if everything is as expected in start up
	def GetInitialPos(self):
		if(self.enabled):
			if(self._simulate == True):
				self._current_state = '111011011100000000000100'
				self._current_state_n = [int(x) for x in list(self._current_state)]
				self._current_goal = '0'
				return self._current_state
			else:
				self.slugsProcess.stdin.write("XGETINIT\n")
				self.slugsProcess.stdin.flush()

				stdout, stderr = "", ""
				#print('.')
				while not "," in stdout:
					stdout += self.slugsProcess.stdout.readline()

				init_state = stdout.replace(">","").strip()
				#print("Init state: {init_state}".format(init_state=init_state))
				self._current_state = init_state.partition(",")[0]
				# save it also as a list of int
				self._current_state_n = [int(x) for x in list(self._current_state)]
				if init_state.partition(",")[2]:
					self._current_goal = init_state.partition(",")[2]
				return self._current_state
		else:
			return ''

	# sets current position, if you need to reset for some reason
	def SetCurrentPos(self, state, sense, action):
		if(self.enabled):
			state_str = format(state, '0%db' %(self._Nself) )

			state_str = state_str[::-1]
			# convert dict of restrictions into a string
			sensed_inputs = ""
			for prop in range(self._Nsens):
				if prop in sense:
					sensed_inputs += "1" if (sense[prop] is True) else "0"
				else:
					sensed_inputs += "."
			state_str = state_str + sensed_inputs # restrictions
			state_str = state_str + format(action, '0%db'%(self._Nout))[::-1] # actions

			#print('....')
			if(self._simulate == False):
				self.slugsProcess.stdin.write("SETPOS\n" + state_str.replace("1","1\n").replace("0","0\n"))
				self.slugsProcess.stdin.flush()
			
			#self.slugsProcess.stdout.flush() # there's some garbage after setpos command
			self.slugsProcess.stdout.readline() # there's some garbage after setpos command
			self.slugsProcess.stdout.readline() # there's some garbage after setpos command
			print('set position')
			self._current_state = state_str
			self._current_state_n = [int(x) for x in list(self._current_state)]
			return self._current_state
		else:
			return ''

	# gets current position, if everything is as expected in start up
	def SetGoal(self, goal=0):
		if(self.enabled):
			if(self._simulate == False):
				self.slugsProcess.stdin.write("XMAKEGOAL\n" + str(goal))
				self.slugsProcess.stdin.flush()
				
			self._current_goal = str(goal)

		
	# in: the sensed robot location & sensors of the available funnels
	def FindNextStep(self, robot_sensed_state, funnel_occupancy_dict):
		if(self.enabled):
			#import pdb; pdb.set_trace()
			# do stuff to create the reversed binary string slugs expects
			sensed_state = format(robot_sensed_state, '0%db' %(self._Nself) )
			sensed_state = sensed_state[::-1]
			#next_state   = sensed_state
			# convert dict of restrictions into a string
			sensed_inputs = ""
			for prop in range(self._Nsens):
				if prop in funnel_occupancy_dict:
					sensed_inputs += "1" if (funnel_occupancy_dict[prop] is True) else "0"
				else:
					sensed_inputs += "."

			sensed_state = sensed_state + sensed_inputs 

			if(self._simulate == True):
				# the simulation for the lab map
				action_list = ['1010,0', '0010,0', '0110,0', '1100,0', '0000,0', '0000,0', '1110,0', '1010,0', '1100,0', \
							   '0000,0', '0000,0', '0000,1', '1001,1', '0000,1', '0000,1', '1100,1', '1100,1', '1010,1', \
							   '0000,1', '0000,1', '0000,1', '0000,1', '0000,1', '0100,2', '0100,2', '1110,2', '0100,2', \
							   '0010,2', '0001,2', '1110,2', '0100,0']
				# emulate what slugs should have given
				trans = sensed_state + action_list[self._state_index]
				self._state_index += 1
				if(self._state_index == len(action_list)):
					self._state_index = 0
			else:
				self.slugsProcess.stdin.write("XMAKETRANS\n" + sensed_state)
				self.slugsProcess.stdin.flush()

				# retrieve the new state
				stdout, stderr = "", ""
				#print('...')
				while not ("," in stdout or "error" in stdout.lower()):
					stdout += self.slugsProcess.stdout.readline()
				trans = stdout.replace(">","").strip()
			
			print("transition to state: {trans}".format(trans=trans))
			if('ERROR' in trans):
				print('transition not followed, consider resetting with SetInitialPos')
				return self.GetNumericState()
			# we should be where we said we need to be
			self._trans_state = trans.partition(",")[0]  #sensed_state
			self._trans_state_n = [int(x) for x in list(self._trans_state)]
				
			if(trans.partition(",")[2]==0 and self._current_goal>0):
				self.SetGoal(goal=0) #slugs is giving different answers second time around :(
				print('SLUGS INTERFACE: resetting goal')
			else:
				self._current_goal = trans.partition(",")[2]

			#print('wrote: %s' %sensed_state)
			next_state_str = self._trans_state[0:self._Nself]
			next_state     = int(next_state_str[::-1], 2)
			next_cmd_str   = self._trans_state[-self._Nout:]
			next_cmd       = int(next_cmd_str[::-1], 2)
			#if(next_cmd == 9):
				#print('got a 9, trying again')
				# try again :(
				#try:
				#	new_goal = int(trans.partition(",")[2])
				#except:
				#	new_goal = 0
				#	import pdb; pdb.set_trace()
				#print('setting goal: %d' %new_goal)
				#self.SetGoal(goal=new_goal)
			#	continue

			#break

			'''
			if(trans.partition(",")[2] == self._current_goal):
				try:
					self._trans_state = trans.partition(",")[0]
					self._trans_state_n = [int(x) for x in list(self._trans_state)]
				except:
					import pdb; pdb.set_trace()
				print('wrote: %s' %sensed_state)
				next_state_str = self._trans_state[0:self._Nself]
				next_state     = int(next_state_str[::-1], 2)
				next_cmd_str   = self._trans_state[-self._Nout:]
				next_cmd       = int(next_cmd_str[::-1], 2)
			else:
				# don't know why but until you set the next goal, it 
				# keeps being in this state and outputting 'stay in place' command
				#goal = int(self._current_goal)
				#new_goal = goal+1 if goal<(self._NGoals-1) else 0
				try:
					new_goal = int(trans.partition(",")[2])
				except:
					new_goal = 0
					import pdb; pdb.set_trace()
				print('setting goal: %d' %new_goal)
				self.SetGoal(goal=new_goal)
				continue

			break
			'''
			
			return next_state, next_cmd
		else:
			return '' , -1
			
	# tell the class the you've actually made the transition
	def MoveNextStep(self):
		if(self.enabled):
			self._current_state = self._trans_state
			self._current_state_n = self._trans_state_n

	# gets the actual numbers from the strings, so it could be used easier from outside
	def GetNumericState(self):
		if(self.enabled):
			#import pdb; pdb.set_trace()
			next_state_str = self._current_state[0:self._Nself]
			next_state     = int(next_state_str[::-1], 2)
			next_cmd_str   = self._current_state[-self._Nout:]
			next_cmd       = int(next_cmd_str[::-1], 2)

			return next_state, next_cmd
		else:
			return -1, -1
	
	def Shutdown(self):
		if(self.enabled):
			if(self._simulate == False):
				self.slugsProcess.terminate()
				self.slugsProcess.wait()
			self.enabled = False
	
if __name__ == "__main__":
	try:
		SI = SlugsInterface('/home/cornell/Tools/slugs_ltl_stack/guy/toy')
		SI.DiscoverInputs()
		SI.DiscoverOutputs()
		SI.DiscoverGoals()
		SI.GetInitialPos()
		
		sense = {0: False, 1: False, 2: False, 3: False}
		SI.FindNextStep(3, sense)
		SI.FindNextStep(6, sense)
		sense = {0: False, 1: False, 2: False, 3: True}
		SI.FindNextStep(6, sense)
		SI.SetInitialPos(6, sense)
		#import pdb; pdb.set_trace()
		'''
		fname = 'lab'
		import networkx as nx
		G = nx.read_gpickle(fname + '.pickle')

		with open(fname + '.label2bit', 'rb') as dbfile:
			map_label_2_bit = dill.load(dbfile)
			map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())

		SI = SlugsInterface(fname + '_r0')
		SI.DiscoverInputs()
		SI.DiscoverOutputs()
		SI.DiscoverGoals()
		SI.GetInitialPos()
		'''
		'''
		cur_state = 0
		sensors = dict(zip(range(SI._Nsens), [False]*SI._Nsens))
		#import pdb; pdb.set_trace()

		for i in range(20):
			#if(i==15):
			#	import pdb; pdb.set_trace()
			cur_state, cur_cmd = SI.GetNumericState()
			cur_state_label = map_bit_2_label[cur_state]
			next_state = -1
			if(cur_cmd==9):
				# stuck, stay in place
				SI.SetGoal(goal=0)
				next_state = cur_state_label
			else:
				for key, val in G[cur_state_label].items():
					if( val['motion'] == cur_cmd):
						next_state = key
						break
			#import pdb; pdb.set_trace()
			SI.FindNextStep(map_label_2_bit[next_state], sensors)
			print('%d) Were in %s (%d), took action %d to get to %s(%d)' \
				  %(i+1, cur_state_label, cur_state, cur_cmd, next_state, map_label_2_bit[next_state]))
			SI.MoveNextStep() # this simulates the fact that we eventually got to where we need
		'''
		'''
		sensors = {}
		i = 0
		cur_state, cur_cmd = SI.GetNumericState()
		cur_state_label = map_bit_2_label[cur_state]
		g_next = -1
		#import pdb; pdb.set_trace()
		while True:
			i += 1
			#cur_state, cur_cmd = SI.GetNumericState()
			if(cur_cmd != 9):
				next_state = -1
				for key, val in G[cur_state_label].items():
					if( val['motion'] == cur_cmd):
						next_state = key
						break
				#import pdb; pdb.set_trace()
				__, cur_cmd = SI.FindNextStep(map_label_2_bit[next_state], sensors)
				g_next = next_state
				print('%d) Were in %s (%d), took action %d to get to %s(%d)' \
					  %(i+1, cur_state_label, cur_state, cur_cmd, next_state, map_label_2_bit[next_state]))
				SI.MoveNextStep()
				cur_state, cur_cmd = SI.GetNumericState()
				cur_state_label = map_bit_2_label[cur_state]
			else:
				cur_state, cur_cmd = SI.FindNextStep(map_label_2_bit[g_next], sensors)
				cur_state_label = map_bit_2_label[cur_state]
				#next_state = cur_state_label
				print('stuck')
			if(i>100):
				break
			
		'''
		'''
		curr_state, action = SI.GetNumericState()
		print('start_state=%d\tcmd=%d' %(curr_state, action))
		next_state, next_cmd = SI.FindNextStep(952, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1338, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1634, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(771, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(451, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(126, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(122, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(761, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1350, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(173, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1292, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(461, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1135, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1726, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(428, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(264, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1266, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1533, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1340, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(968, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(978, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(552, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1541, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		next_state, next_cmd = SI.FindNextStep(1537, sensors)
		print('next_state=%d\tnext_cmd=%d' %(next_state, next_cmd))
		SI.MoveNextStep()
		'''
	finally:
		SI.Shutdown()
	#import pdb; pdb.set_trace()
	
	
	
	
	
	
	
	