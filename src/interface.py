#!/usr/bin/env python
'''
############### NLR: AR.Drone Keyboard Interface ###############

Filename:       interface.py
Description:    This interface is a control interface for the AR.Drone 1/2.
                It depends on the ardrone_autonomy driver, which contains
                the AR.Drone SDK and implements the basic communication. This
                interface can be used to fly the AR.Drone
By:             Camiel Verschoor
Created:        22-10-2012

############### NLR: AR.Drone Keyboard Interface ###############
'''

# Libraries
import roslib; roslib.load_manifest('ardrone_interface')
import rospy
import pygame
import std_srvs.srv
import time
from subprocess import Popen
from pygame.locals import * 
from std_msgs.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from tld_msgs.msg import BoundingBox
from ardrone_autonomy.msg import Navdata
from tld_msgs.msg import Target
from datetime import datetime


class Interface():
    ''' User Interface for controlling the AR.Drone '''

    def __init__(self):
        ''' Constructor for setting up the User Interface '''
    	# Initialize pygame
        pygame.init()
        
        # Initialize Clock
        self.clock = pygame.time.Clock()

        # Setup the main screen
        self.resolution = (640, 460) # This is the screen size of AR.Drone 2. Works also for AR.Drone 1
        self.screen     = pygame.display.set_mode( self.resolution )
        pygame.display.set_caption( 'NLR: AR.Drone Keyboard Interface' )

        # Setup the background
        self.background = pygame.Surface( self.screen.get_size() )
        self.background = self.background.convert()
        self.background.fill( (255, 255, 255) )

        # Setup logo
        self.logo           = pygame.image.load( roslib.packages.get_pkg_dir('ardrone_interface')+ "/images/logo.png" ).convert()
        self.logo_rect      = self.logo.get_rect()
        self.logo_rect.left = 0
        self.logo_rect.top  = 360
	

        self.background.blit( self.logo, self.logo_rect )
        self.screen.blit( self.background, (0,0) )
        pygame.display.flip()
	
        # ROS Settings
        self.publisher_land           = rospy.Publisher(  '/ardrone/land',      Empty )
        self.publisher_takeOff        = rospy.Publisher(  '/ardrone/takeoff',   Empty )
	self.publisher_reset	      = rospy.Publisher(  '/ardrone/reset',     Empty ) #edited by Ardillo make a reset possible after over-tilt
        self.publisher_parameters     = rospy.Publisher(  '/cmd_vel',           Twist )
        self.publisher_tracking_box   = rospy.Publisher(  '/tld_gui_bb',        Target ) #merged from CamielV's repo
	self.publisher_reset_tracker  = rospy.Publisher(  '/tld_gui_cmds', String)
        self.subscriber_camera_front  = rospy.Subscriber( '/ardrone/front/image_raw',  Image, self.__callback_camera ) # Front image
        self.subscriber_camera_bottom = rospy.Subscriber( '/ardrone/bottom/image_raw', Image, self.__callback_camera ) # Bottom image
        self.subscriber_navdata       = rospy.Subscriber( '/ardrone/navdata', Navdata, self.__callback_navdata ) # Navdata
	self.subscriber_imu           = rospy.Subscriber( '/ardrone/imu', Imu, self.__callback_imu ) # Imu
        self.subscriber_tracker       = rospy.Subscriber( '/tld_tracked_object', BoundingBox, self.__callback_tracker ) # Tracker      
        self.parameters               = Twist()
        rospy.init_node( 'interface' )

        # AR.Drone Variables
        self.airborne = False
        self.speed    = 0.2
        self.image    = None
	self.manual_flightmode = True
	self.confidence = None 	#confidence variable by Ardillo, has to be in front of ROS Settings.
	self.header_seq = None
	self.header_sec = None
	self.old_seq = None
	self.battery_percent = None
	self.altitude = None
        self.init_width = None
        self.init_height = None

        # Tracking box
        self.returning_tracking_box = pygame.Rect(641, 461, 1, 1)
	# resolution videofeed = 640 x 360
	self.center_box_width = 128  #64 #128 #192 #256
	self.center_box_height = 92 #46 #92  #138 #184
	self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )
	self.key_5 = False
	self.key_6 = False
	self.key_7 = False
	self.key_8 = False
	self.key_9 = False
	self.key_0 = False


        # Tracking box outside of screen
        self.tracking = False
        self.tracking_box = None

        # Select box
        self.selected    = False
        self.click_loc   = None
        self.release_loc = None
	self.select_image = None
        

    def __del__(self):
        ''' Destructor of the User Interface'''
        pygame.quit()

    def run(self):
        ''' Main loop, which refreshes the screen and handles User Input '''
        print "Starting NLR: AR.Drone Keyboard Interface"
        done = False

	# resolution videofeed = 640 x 360
	self.center_box_width = 64  #64 #128 #192 #256
	self.center_box_height = 46 #46 #92  #138 #184
	self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )


        while not(done):
	    # measurementbox has to change
	    if self.key_5 and self.center_box_width > 1 and self.center_box_height > 1:
		self.center_box_width -= 1
		self.center_box_height -= 1
                print "width:" , self.center_box_width , "height:" , self.center_box_height			
	    elif self.key_6 and self.center_box_width <= (640 -1) and self.center_box_height <= (360 -1):
		self.center_box_width += 1
		self.center_box_height += 1			
                print "width:" , self.center_box_width , "height:" , self.center_box_height
	    elif self.key_7 and self.center_box_width > 1:
		self.center_box_width -= 1
                print "width:" , self.center_box_width , "height:" , self.center_box_height
	    elif self.key_8 and self.center_box_width <= (640 -1):
		self.center_box_width += 1
                print "width:" , self.center_box_width , "height:" , self.center_box_height
	    elif self.key_9 and self.center_box_height > 1:
		self.center_box_height -= 1
                print "width:" , self.center_box_width , "height:" , self.center_box_height
	    elif self.key_0 and self.center_box_height <= (360 -1):
		self.center_box_height += 1
                print "width:" , self.center_box_width , "height:" , self.center_box_height
	    self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )



            for event in pygame.event.get():
                # Check if window is quit
                if event.type == pygame.QUIT:
                    done = True
                    break
                # Check if mousebutton is pressed
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        if not(self.tracking):
                            self.selected = True
                            self.click_loc = event.pos
			    self.select_image = self.image
                # Check if mousebutton is released
                elif event.type == pygame.MOUSEBUTTONUP:
                    # Left mouse button
                    if event.button == 1:
                        if not(self.tracking):
                            self.selected = False
                            self.release_loc = event.pos
                            self.__updateSelectBox()
                # Check if mouse is moved
                elif event.type == pygame.MOUSEMOTION:
                    if not(self.tracking) and self.selected:
                        self.release_loc = event.pos
                        self.__updateSelectBox()

                # Check if key is pressed
                elif event.type == pygame.KEYDOWN:
                    if   event.key == pygame.K_UP:
                        self.parameters.linear.x = self.speed
                    elif event.key == pygame.K_LEFT:
                        self.parameters.linear.y = self.speed
                    elif event.key == pygame.K_DOWN:
                        self.parameters.linear.x = -self.speed
                    elif event.key == pygame.K_RIGHT:
                        self.parameters.linear.y = -self.speed
                    elif event.key == pygame.K_w:
                        self.parameters.linear.z = 3 *self.speed
                    elif event.key == pygame.K_a:
                        self.parameters.angular.z = 2 * self.speed
                    elif event.key == pygame.K_s:
                        self.parameters.linear.z = 3 * -self.speed
                    elif event.key == pygame.K_d:
                        self.parameters.angular.z = 2 * -self.speed
                    elif event.key == pygame.K_c:
                        self.__toggleCam()

		    elif event.key == pygame.K_r: #edited by Ardillo making reset function
			self.__reset()
			
		    elif event.key == pygame.K_1:  #edited by Ardillo making measurementbox realtime adjustable
			self.center_box_width = 64  
			self.center_box_height = 46 
			self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )
		    elif event.key == pygame.K_2:
			self.center_box_width = 128 
			self.center_box_height = 92 
			self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )
		    elif event.key == pygame.K_3:
			self.center_box_width = 192 
			self.center_box_height = 138
			self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )
		    elif event.key == pygame.K_4:
		    	self.center_box_width = 256 
			self.center_box_height = 184
			self.center_box = pygame.Rect((320-(self.center_box_width/2)), (180-(self.center_box_height/2)), self.center_box_width, self.center_box_height )
		    elif event.key == pygame.K_5:
			self.key_5 = True
		    elif event.key == pygame.K_6:
			self.key_6 = True
		    elif event.key == pygame.K_7:
			self.key_7 = True
		    elif event.key == pygame.K_8:
			self.key_8 = True
		    elif event.key == pygame.K_9:
			self.key_9 = True
		    elif event.key == pygame.K_0:
			self.key_0 = True

                    elif event.key == pygame.K_RETURN: #merged from CamielV's repo
                        if self.tracking_box:
                            self.tracking = True
                            self.__sendTrackingBox()
		    elif event.key == pygame.K_t: # tracker reset by Ardillo --not working properly-- TODO
			if self.tracking == True:
			    print"Resetting tracker"
			    self.tracking_box = None
			    self.tracking = False
			str = "r"
			self.publisher_reset_tracker.publish(String(str))

                    elif event.key == pygame.K_MINUS:
                        self.__switchSpeed( -0.01 ) #edited by Ardillo making it more sensible
                        print self.speed
                    elif event.key == pygame.K_EQUALS:
                        self.__switchSpeed( 0.01 ) #edited by Ardillo making it more sensible
                        print self.speed
		    elif event.key == pygame.K_b:
			print "Battery:", self.battery_percent
                    elif event.key == pygame.K_SPACE:		    			
                        if self.airborne:
                            self.__land()
                            self.airborne = False
                        else:
                            self.__takeOff()
                            self.airborne = True
		    elif event.key == pygame.K_m: # Boolean toggle by Ardillo
			self.manual_flightmode = not self.manual_flightmode
			print "Manual_flightmode =",self.manual_flightmode
			if self.manual_flightmode == False:
			    self.__trackObject()
			    print "Manual_flightmode =",self.manual_flightmode
			
                # Check if key is released.
                elif event.type == pygame.KEYUP:
                    if   event.key == pygame.K_UP:
                        self.parameters.linear.x = 0
                    elif event.key == pygame.K_LEFT:
                        self.parameters.linear.y = 0
                    elif event.key == pygame.K_DOWN:
                        self.parameters.linear.x = 0
                    elif event.key == pygame.K_RIGHT:
                        self.parameters.linear.y = 0
                    elif event.key == pygame.K_w:
                        self.parameters.linear.z = 0
                    elif event.key == pygame.K_a:
                        self.parameters.angular.z = 0
                    elif event.key == pygame.K_s:
                        self.parameters.linear.z = 0
                    elif event.key == pygame.K_d:
                        self.parameters.angular.z = 0
		    elif event.key == pygame.K_5:
			self.key_5 = False
		    elif event.key == pygame.K_6:
			self.key_6 = False
		    elif event.key == pygame.K_7:
			self.key_7 = False
		    elif event.key == pygame.K_8:
			self.key_8 = False
		    elif event.key == pygame.K_9:
			self.key_9 = False
		    elif event.key == pygame.K_0:
			self.key_0 = False

            self.publisher_parameters.publish( self.parameters )
            self.__draw()
            self.clock.tick(30)

    def __draw(self):
        ''' Draws the camera feed on the screen '''
        if self.image == None:
            return
 	image = pygame.image.fromstring( self.image.data, (self.image.width, self.image.height), "RGB" )
        self.background.blit( image, (0, 0) )
	if self.old_seq == self.header_seq: # don't show old rectangles
	    self.returning_tracking_box = pygame.Rect(641, 461, 1, 1)	
	else:
	    pygame.draw.rect( self.background, (0, 0, 255), self.returning_tracking_box, 2 )
	self.old_seq = self.header_seq
        if self.tracking_box and self.tracking == False: # Made some changes so it doesn't stay drawed while tracking.
            pygame.draw.rect( self.background, (0, 255, 0), self.tracking_box, 2 ) #merged from CamielV's repo
       	pygame.draw.rect( self.background, (100, 100, 100), self.center_box, 1 ) 
        self.screen.blit( self.background, (0, 0) )
        pygame.display.flip()

    def __updateSelectBox(self):
        if not(self.click_loc and self.release_loc):
	    print"error, returning. You're mouse is broken"            
	    return
	    

        x1, y1 = self.click_loc
        x2, y2 = self.release_loc	

        # Determining height and width of rect
        width_rect  = abs(x1 - x2)
        height_rect = abs(y1 - y2)

        # Determining top left
        min_x = x1
        min_y = y1
        if x1 < x2:
            if y1 < y2:
                pass
            else:
                min_y = y2
        else:
            if y1 < y2:
                min_x = x2
            else:
                min_x = x2
                min_y = y2
        if min_x > self.resolution[0]:
            return
        if min_y > self.resolution[1] - 100:
            return
        if width_rect + min_x > self.resolution[0]:
            return
        if height_rect + min_y > self.resolution[1] - 101:
            return
        self.tracking_box = pygame.Rect(min_x, min_y, width_rect, height_rect)
	#print"tracking_box", self.tracking_box

    def __sendTrackingBox(self):
        target = Target()
        target.bb.x          = self.tracking_box.x
        target.bb.y          = self.tracking_box.y
        target.bb.width      = self.tracking_box.width
        target.bb.height     = self.tracking_box.height
        target.bb.confidence = 1.0
	#print "Target  =" , target
        target.img           = self.select_image
        self.publisher_tracking_box.publish( target )
	print "Bounding box send" 
        self.clock.tick(100)
	self.tracking_box = None
	self.tracking = False
	

    def __toggleCam(self):
        ''' Switches between camera feeds of the AR.Drone '''
        rospy.wait_for_service( 'ardrone/togglecam' )
        try:
            toggle = rospy.ServiceProxy( 'ardrone/togglecam', std_srvs.srv.Empty )
            toggle()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def __takeOff(self):
        ''' Take off signal for AR.Drone '''
        print "Taking off"
        self.publisher_takeOff.publish( Empty() )

    def __land(self):
        ''' Landing signal for AR.Drone '''
        print "Landing"
        self.publisher_land.publish( Empty() )

    def __callback_camera(self, raw_image):
        ''' Callback function for the camera feed '''
        if self.tracking or not(self.selected):
            self.image = raw_image

    def __callback_tracker(self, tracking_box):
        ''' Callback function for the rectangle'''
	self.returning_tracking_box = pygame.Rect( tracking_box.x, tracking_box.y, tracking_box.width, tracking_box.height )
	self.confidence = tracking_box.confidence # got the confidence variable, for future use. By Ardillo
	self.header_seq = tracking_box.header.seq
	self.header_sec = tracking_box.header.stamp.secs	

    def __callback_navdata(self, navdata):
        ''' Callback function for the camera feed '''
        self.battery_percent = navdata.batteryPercent
	self.altitude = navdata.altd
   
    def __callback_imu(self, imudata):
	''' Callback for the imu data feed ''' # feedback from IMU, used for making PID controller
	self.orientation_x = abs(imudata.orientation.x) * 10
	self.orientation_x = 1 - self.orientation_x
	if self.orientation_x < 0:
	    self.orientation_x = 0
	self.orientation_y = abs(imudata.orientation.y)	* 10
	self.orientation_y = 1 - self.orientation_y
	if self.orientation_y < 0:
	    self.orientation_y = 0  
	self.orientation_compass = imudata.orientation.z # from 0--1=-1--0
	self.linear_acceleration_x = imudata.linear_acceleration.x 
	self.linear_acceleration_y = imudata.linear_acceleration.y 
	self.linear_acceleration_z = imudata.linear_acceleration.z 
	#print "x:" , self.linear_acceleration_x , " y:" , self.linear_acceleration_y , " z:" , self.linear_acceleration_z
	#print "orientation x:" , self.orientation_x , " y:" , self.orientation_y
	#print "compass:" , self.orientation_compass

    def __switchSpeed( self, speed ):
        new_speed = self.speed + speed
        if new_speed >= -1 and new_speed <= 1:
            self.speed = new_speed

    def __reset(self):				# edited by Ardillo making reset function
	''' Reset signal for AR.Drone '''
	print "Resetting"
	self.publisher_reset.publish( Empty() )


	# making an automated flight procedure by Ardillo, NO CONTROLS POSSIBLE EXCEPT EMERGENCY RESET
    def __trackObject(self):			
	''' Track the target '''
	print "In autonomous_flightmode"
	print "ARdrone says: 'I can handle it myself'"
	done = False
	firstTime = True
	offset = 20				# in te stellen voor elevator-as
	self.goLeft = False
	self.strafeLeft = False
	self.strafeRight = False
	self.goRight = False
        self.goBackward = False
        self.goForward = False
	self.goUp = False
	self.goDown = False
        self.startTime = datetime.now()
	self.noTrackTime = datetime.now()
	self.Left_timer = 0
	self.Right_timer = 0
	while not(done):
		
		# Fallback if object is lost longer than 1 second
		if not firstTime:
	    	    if self.startTime < self.noTrackTime:
		    	if abs(self.startTime - self.noTrackTime ).seconds > 1:			
	            	    print "no Track for" , abs(self.startTime - self.noTrackTime ).seconds , "seconds"
                    	    self.parameters.linear.x = 0
	 	    	    self.parameters.linear.y = 0
		    	    self.parameters.linear.z = 0
			    self.parameters.angular.z = 0
	    	    	    self.publisher_parameters.publish( self.parameters )

		# check location of Bounding box.
	    	if self.returning_tracking_box.x <= 640 and self.returning_tracking_box.y <= 460:
		    if firstTime:
			self.init_width = self.returning_tracking_box.width
			self.init_height = self.returning_tracking_box.height
			firstTime = False
			self.start_altitude = self.altitude
			self.heading_at_start = self.orientation_compass

		    self.center_tracking_box_x = self.returning_tracking_box.x + (self.returning_tracking_box.width / 2)
		    self.center_tracking_box_y = self.returning_tracking_box.y + (self.returning_tracking_box.height / 2)
		    
		    	
		    if self.old_seq != self.header_seq:	# only when tracking node publishes a new image
 			self.startTime = datetime.now()
			# # A factor from 0 - 1 for being not centered. Used for PID controller (P proportional)
			self.factor_x = float(abs(self.center_tracking_box_x - 320))/320
			self.factor_y = float(abs(self.center_tracking_box_y - 230))/230
			#print "factor X:" , self.factor_x , "factor Y:" , self.factor_y			
			
			## Main steering signals

			#testing code with compass
			'''print "heading" , self.heading_at_start
			if self.heading_at_start < 0:
			    # right side of scale
			    if self.orientation_compass < self.heading_at_start:
  		                print "turn Left"
			    	self.goLeft = True
			    	self.integral_factor = self.orientation_x
		            	self.parameters.angular.z = 2 * self.speed * self.factor_x * self.integral_factor
	                    	self.publisher_parameters.publish( self.parameters )
			    elif self.orientation_compass > self.heading_at_start:
				print "turn Right"
			    	self.goRight = True
			    	self.integral_factor = self.orientation_x
		            	self.parameters.angular.z = 2 * -self.speed * self.factor_x * self.integral_factor
                            	self.publisher_parameters.publish( self.parameters )
			elif self.heading_at_start > 0:
			    # left side of scale
			    if self.orientation_compass < self.heading_at_start:
				print "turn Right"
			    	self.goRight = True
			    	self.integral_factor = self.orientation_x
		            	self.parameters.angular.z = 2 * -self.speed * self.factor_x * self.integral_factor
                            	self.publisher_parameters.publish( self.parameters )
			    elif self.orientation_compass > self.heading_at_start:			     
  		                print "turn Left"
			    	self.goLeft = True
			    	self.integral_factor = self.orientation_x
		            	self.parameters.angular.z = 2 * self.speed * self.factor_x * self.integral_factor
	                    	self.publisher_parameters.publish( self.parameters )
			else:
			    self.parameters.angular.z = 0
                            self.publisher_parameters.publish( self.parameters ) '''
			
 			# testing code (steering commands for Roll)
			if self.Left_timer > 3:
		            print "strafe Left"
			    self.strafeLeft = True
			    self.Left_timer = 0
			    self.integral_factor = self.orientation_x
		            self.parameters.linear.y = self.speed * self.factor_x * self.integral_factor
	                    self.publisher_parameters.publish( self.parameters )
			elif self.Right_timer > 3:
			    print "strafe Right"
			    self.strafeRight = True
			    self.Right_timer = 0
			    self.integral_factor = self.orientation_x
		            self.parameters.linear.y = -self.speed * self.factor_x * self.integral_factor
                            self.publisher_parameters.publish( self.parameters )
			else:
			    self.parameters.linear.y = 0
                            self.publisher_parameters.publish( self.parameters )
			
			# Basic steering commands Yaw
                        if self.center_tracking_box_x < self.center_box.x:
  		            print "turn Left"
			    self.goLeft = True
			    self.Left_timer += 1
		            self.parameters.angular.z = 2 * self.speed * self.factor_x 
	                    self.publisher_parameters.publish( self.parameters )
			elif self.center_tracking_box_x > (self.center_box.x + self.center_box.width):
			    print "turn Right"
			    self.goRight = True
			    self.Right_timer += 1
		            self.parameters.angular.z = 2 * -self.speed * self.factor_x 
                            self.publisher_parameters.publish( self.parameters )
			else:
			    self.parameters.angular.z = 0
                            self.publisher_parameters.publish( self.parameters )

			# Basic steering commands Pitch
		    	if self.center_tracking_box_y < self.center_box.y:
		            print "go Forward"
			    self.goForward = True
			    self.integral_factor = self.orientation_y
		            self.parameters.linear.x = 0.5 * self.speed * self.factor_y * self.integral_factor
                            self.publisher_parameters.publish( self.parameters )
			elif self.center_tracking_box_y > (self.center_box.y + self.center_box.height):
		            print "go Backward"
			    self.goBackward = True
			    self.integral_factor = self.orientation_y
		            self.parameters.linear.y = - self.speed * self.factor_x * self.integral_factor
			    self.publisher_parameters.publish( self.parameters )
			else:
			    self.parameters.linear.x = 0
                            self.publisher_parameters.publish( self.parameters )	

			# Basic steering commands elevator
			if (self.returning_tracking_box.width + offset) < self.init_width or (self.returning_tracking_box.height + offset) < self.init_height:
			    print "go Up"
			    self.goUp = True
			    self.parameters.linear.z = self.speed  * self.confidence
			    self.publisher_parameters.publish( self.parameters )
			elif (self.returning_tracking_box.width - offset) > self.init_width or (self.returning_tracking_box.height - offset) > self.init_height:
			    print "go Down"
			    self.goDown = True
			    self.parameters.linear.z = -self.speed * self.confidence
			    self.publisher_parameters.publish( self.parameters )
			else:			    	
			    self.parameters.linear.z = 0
		            self.publisher_parameters.publish( self.parameters )
			
			## Correction if x-location object is in the center_box
			if self.center_tracking_box_x > self.center_box.x and self.center_tracking_box_x < (self.center_box.x + self.center_box.width):
			    # Correction roll and yaw axe
			    if self.strafeLeft:
				print "Correct Right"
				self.strafeLeft = False
				self.parameters.linear.y = -0.3
				self.publisher_parameters.publish( self.parameters )
			    if self.strafeRight:
				print "Correct Left"
				self.strafeRight = False
				self.parameters.linear.y = 0.3
				self.publisher_parameters.publish( self.parameters )
			    self.parameters.linear.y = 0
			    self.parameters.angular.z = 0
			    self.publisher_parameters.publish( self.parameters )
			
			## Correction if x-location object is in the center_box
			if self.center_tracking_box_y > self.center_box.y and self.center_tracking_box_y < (self.center_box.y + self.center_box.height):
			    # Correction pitch axe
			    if self.goForward:
				print "correct Backward"
				self.goForward = False
				self.parameters.linear.x = -0.3
				self.publisher_parameters.publish( self.parameters )
			    if self.goBackward:
				print "correct Forward"
				self.goBackward = False
				self.parameters.linear.x = 0.3
				self.publisher_parameters.publish( self.parameters )
			    self.parameters.linear.x = 0
			    self.publisher_parameters.publish( self.parameters )
			
			# Correction altitude compared to starting altitude	
		    	if self.altitude < self.start_altitude - 75:
		    	    print "to Low, correcting myself"
		    	    self.parameters.linear.z = self.speed 
		    	    self.publisher_parameters.publish( self.parameters )
		    	elif self.altitude > self.start_altitude + 75:
		    	    print "to High, correcting myself"
		    	    self.parameters.linear.z = -self.speed 	
		    	    self.publisher_parameters.publish( self.parameters )
		else:
		    self.noTrackTime = datetime.now() 

					    

	        for event in pygame.event.get():
        	    # Check if window is quit
        	    if event.type == pygame.QUIT:
        	        done = True
        	        break
			break
        	    # Check if key is pressed
        	    elif event.type == pygame.KEYDOWN:
		        if  event.key == pygame.K_m:
			    self.parameters.linear.x = 0
			    self.parameters.linear.y = 0
			    self.parameters.linear.z = 0
			    self.parameters.angular.z = 0
			    self.publisher_parameters.publish( self.parameters )		    
			    print "Back to manual_flightmode"
			    self.manual_flightmode = not self.manual_flightmode
			    return
		        elif event.key == pygame.K_r: 
			    self.__reset()
                        elif event.key == pygame.K_b:
			    print "Battery:", self.battery_percent
	                elif event.key == pygame.K_MINUS:
        	            self.__switchSpeed( -0.01 ) #edited by Ardillo making it more sensible
                            print self.speed
                        elif event.key == pygame.K_EQUALS:
                            self.__switchSpeed( 0.01 ) #edited by Ardillo making it more sensible
                            print self.speed

	        self.__draw()


if __name__ == '__main__':
    ''' Starts up the software '''
    print '\n---> Starting up driver!\n'
    ardrone_driver = Popen( ['rosrun', 'ardrone_autonomy', 'ardrone_driver'])
    #opentld        = Popen( ['roslaunch', 'tld_tracker', 'ros_tld_tracker.launch'])
    print '\n---> Starting up NLR: AR.Drone Keyboard Inferface!\n'
    GUI = Interface()
    try:
        GUI.run()
    except Exception as e:
        print "ERROR:", e
    ardrone_driver.kill()
    #ros_tld_tracker.kill()
    print '\n---> Shutting down driver!\n'
    print '\n---> Ended Successfully!\n'
