#!/usr/bin/env python

class PIDAutoTuning:

	def __init__(self,alpha_p,alpha_d,kp,kd,ki,flag):

		self.alpha_p=alpha_p
		self.alpha_d=alpha_d
		self.kp=kp;
		self.kd=kd;
		self.ki=ki;
		self.prev_input=0
		self.prev_output=0
		self.prev_error=0
		self.i_error=0
		self.yaw_flag=flag

	def AutoTuning(self,error,der_error,cur_input,cur_output):

		dy=cur_output-self.prev_output
		du=cur_input-self.prev_input	
		dsystem=dy/du

		grad_p=self.alpha_p*error*dsystem*error
		grad_d=self.alpha_d*error*dsystem*der_error

		self.kp+=grad_p
		self.kd+=grad_d

		if not self.yaw_flag:
			if self.kp<0.2:
				self.kp=0.2
			if self.kd<0.2:
				self.kd=0.2
		
			if self.kp>0.8:
				self.kp=0.8
			if self.kd>0.7:
				self.kd=0.7
		if self.yaw_flag:
			if self.kp<0.1:
				self.kp=0.1
			if self.kp>0.5:
				self.kp=0.5
			if self.kd>0.2:
				self.kd=0.2
			if self.kd<0.05:
				self.kd=0.05

		self.prev_input=cur_input
		self.prev_output=cur_output


	def PIDController(self,cur_pos,target,dt):

		error=target-cur_pos
		self.error=error
		#if abs(error)<0.01:
		#	error=0
		self.i_error+=error*dt
		if self.i_error>200:
			self.i_error=200
		if self.i_error<-200:
			self.i_error=-200
		der_error=(error-self.prev_error)/dt

		system_input=self.kp*error+self.kd*der_error+self.ki*self.i_error

		if system_input<-1:
			system_input=-1
		if system_input>1:
			system_input=1
	

		if self.prev_input!=system_input:
			self.AutoTuning(error,der_error,system_input,cur_pos)

		self.prev_error=error

		return system_input



