import gym
import pybullet as p
import pybullet_data
import numpy as np
import random, time, math
import os


class SimpleDrivingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, render = True,
                    basePath = os.getcwd(), movingCar = 'racecar'):
        """Inherited from gym.Env
        
        Arguments:\n
            - render -- True if the arena should be graphically rendered, else False
            - basePath -- basePath of where the code is run from
            - movingCar -- The car to park(one of 'racecar' or 'husky')

        List of Available Functions:\n
            - render() -- Not useful as rendering occurs as a parameter in gym.make()
            - reset() -- Reset the simulation
            - step(action) -- Take a step
            - seed() -- Returns the seed for the environment
            - close() -- Close the environment
        """
        self.car = None
        self.goal = None
        self.done = False
        self.parking_location = -1
        self.dist = 0
        self.vector = 0
        # TODO: Move p.connect() to env.render and env.reset, so that there is an option for choosing DIRECT and GUI modes
        if render:
            self.client = p.connect(p.GUI)
            time.sleep(1./240.)
        else:
            self.client = p.connect(p.DIRECT)
            time.sleep(1./240.)
        self.theta = 0

        self.numCarsInOneRow = 8
        self.slot1 = None
        self.slot2 = None

        self.left_cars = [None] * self.numCarsInOneRow
        self.right_cars = [None] * self.numCarsInOneRow
        self.left_only = False
        self.la = [1] * self.numCarsInOneRow
        self.ra = [2] * self.numCarsInOneRow

        self.basePath = basePath
        self.movingCar = movingCar

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        
        # self.reset()

    def render(self, mode='human'):
        """Not useful in its current form, as render is a parameter passed in gym.make"""
        # rendering
        p.stepSimulation(self.client)
        # p.stepSimulation()
        time.sleep(1./240.)

        # step simulation

    def reset(
        self,
        left_parking_location = -1,
        right_parking_location = -1,
        shouldParkedCarsBeLoaded = True,
        shouldParkedCarsBeMovedBackAndForth = False,
        left_only = False,
        ):
        """
        Resets the simulation.

        Arguments:\n
            - left_parking_location -- Value between 0 to number of Cars in each row
            - right_parking_location -- Value between 0 to number of Cars in each row
            A value should be provided to only one of these 2. Left will pe given preference if both have a value.\
            In case neither are given, a parking slot will be alloted randomly
            - shouldParkedCarsBeLoaded -- True if parked cars should be loaded, otherwise False
            - shouldParkedCarsBeMovedBackAndForth -- True if the parked cars should be moved back and forth
            - left_only -- True if parked cars should only be present on the left, otherwise False
        
        Returns:\n
            - obs -- List of 14 observations
        """
        if shouldParkedCarsBeMovedBackAndForth:
            assert shouldParkedCarsBeLoaded, "Cars must be loaded to move back and forth"
        self.shouldParkedCarsBeMovedBackAndForth = shouldParkedCarsBeMovedBackAndForth
        self.left_only = left_only
        # reset function to be made for muliple-agents
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -10)
        # Loading the plane
        p.loadURDF(os.path.join(self.basePath, "rsc/arena.urdf"), basePosition=[3.7, 6.0, 0.005],useFixedBase=10)

        # Loading Boundaries
        p.loadURDF(os.path.join(self.basePath, "rsc/boundary1.urdf"), basePosition=[0.06,6.1,0.75], useFixedBase=10)
        p.loadURDF(os.path.join(self.basePath, "rsc/boundary2.urdf"), basePosition=[3.7,12.2,0.75], useFixedBase=10)
        p.loadURDF(os.path.join(self.basePath, "rsc/boundary1.urdf"), basePosition=[7.2,6.1,0.75], useFixedBase=10)
        p.loadURDF(os.path.join(self.basePath, "rsc/boundary3.urdf"), basePosition=[0.9,0.1,0.75], useFixedBase=10)
        p.loadURDF(os.path.join(self.basePath, "rsc/boundary3.urdf"), basePosition=[6.45,0.1,0.75], useFixedBase=10)


        p.addUserDebugLine([1.7,0.2,0.02], [1.7,12.0,0.02], [1,1,1],10)
        if self.left_only is not True:
            p.addUserDebugLine([5.6,0.2,0.02], [5.6,12.0,0.02], [1,1,1],10)
        
        self.__loadCarAndSlots(shouldParkedCarsBeLoaded, left_parking_location, right_parking_location)
        # Reload the plane and car
        self.t = Car(self.client, baseOrientationEuler=[0, 0, math.pi/2], carType = self.movingCar)
        self.car = self.t.car
        # Set the goal to a target(co-ordinate points of parking for each car)

        # Get observation to return
        car_ob, self.vector= self.t.get_observation()
        dist = self.__getDistances(self.car)
        ob = (list(car_ob)+dist)
        # print(dist)
        # get_car_initial_observation
        return ob

    def __getDistances(self, car):
        """Helper Function to get the distances of the car from obstacles"""
        pos = p.getBasePositionAndOrientation(car, physicsClientId=self.client)
        pos3=[0,0,0]
        pos1 = pos[0]
        dist = np.zeros(self.numCarsInOneRow)
        pos2 = p.getEulerFromQuaternion(pos[1])
        self.theta = self.theta + pos2[2] - pos3[2]
        pos3 = pos2
        # dist = self.__fun(pos1[0], pos1[1], pos1[2], self.theta) # For Husky
        dist = self.__fun(pos1[0], pos1[1], 0.4, self.theta) # for RaceCar
        return dist

    def __fun(self,pos1,pos2,pos3,thetha):
        """Helper function to get distances"""
        r=7
        x=[0 for i in range(8)]
        t=[0 for i in range(8)]
        dist=np.zeros(8)
        for i in range(8):
            x[i]=p.rayTest([pos1,pos2,pos3],[pos1+r*math.cos(math.pi*i/4+thetha),pos2+r*math.sin(math.pi*i/4+thetha),pos3])

            #print(x[i])
            x[i]=x[i][0]
            x[i]=list(x[i])
            #print(x[i])
            t[i]=list(x[i][3])
            #print(t[i])
            # p.addUserDebugLine([pos1+0.2*math.cos(thetha+math.pi/2),pos2,pos3],[pos1+r*math.cos(thetha+math.pi*i/4),pos2+r*math.sin(thetha+math.pi*i/4),pos3],[1,0,1])
            if(x[i][0]==-1 or 0):
                #p.addUserDebugLine([pos1,pos2,pos3+0.7],[pos1+r*math.cos(0+math.pi*i/6+thetha),pos2+r*math.sin(0+math.pi*i/6+thetha),pos3],[1,0,1])
                dist[i]=7
                #print(dist[i])
            else:
                #p.addUserDebugLine([pos1,pos2,pos3+0.7],t[i],[1,0,0])
                dist[i]=np.sqrt(pow(t[i][0]-pos1,2)+pow(t[i][1]-pos2,2)+pow(t[i][2]-pos3,2))
        dist[0]-=0.16
        dist[1]+=0.09
        dist[7]+=0.09
        dist[2]+=0.12
        dist[6]+=0.13
        dist[3]+=0.19
        dist[5]+=0.19
        dist[4]+=0.20
        # dist[5]+=0.09
        distance = [round(i,3) for i in dist]
        return distance

    def __loadCarAndSlots(self, shouldParkedCarsBeLoaded, left_parking_location, right_parking_location):
        """Loading Car and Slots"""
        pos_choices = np.arange(0, self.numCarsInOneRow)
        slot_side = 'none'
        if left_parking_location == -1 and right_parking_location == -1:
            self.parking_location = self.slot1
            a = random.choice(pos_choices)
            b = random.choice(pos_choices)
        elif left_parking_location != -1:
            assert left_parking_location >= 0 and left_parking_location < self.numCarsInOneRow - 1, f"The parking slot must be one of 0 to {self.numCarsInOneRow}"
            self.parking_location = left_parking_location
            a = left_parking_location
            b = random.choice(pos_choices)
            slot_side = 'left'
        else:
            assert right_parking_location >= 0 and right_parking_location < self.numCarsInOneRow - 1, f"The parking slot must be one of 0 to {self.numCarsInOneRow}"
            self.parking_location = right_parking_location
            a = random.choice(pos_choices)
            b = right_parking_location
            print("b:", b)
            slot_side = 'right'
        
        if shouldParkedCarsBeLoaded:
            for i in range (8):
                if i == a:
                    continue
                if self.shouldParkedCarsBeMovedBackAndForth:
                    vel = random.randint(2, 5)
                    force = random.randint(50, 120)
                    self.left_cars[i] = Car(self.client, basePosition=[0.8, 0.8+1.5*i, 0.1], val=vel, maxForce=force)
                else:
                    self.left_cars[i] = p.loadURDF("husky/husky.urdf", basePosition=[0.8, 0.8+1.5*i, 0.1], useFixedBase = True)
                p.stepSimulation()
                time.sleep(1/240)

        a1 = [0.2, 0.2+1.5*a, 0]
        b1 = [0.2, 0.2+1.5*(a+1), 0]
        c1 = [1.7, 0.2+1.5*a, 0]
        d1 = [1.7, 0.2+1.5*(a+1), 0]
        self.slot1 = np.array([a1[:2], b1[:2], c1[:2], d1[:2]])

        a1[2] = 0.02
        b1[2] = 0.02
        c1[2] = 0.02
        d1[2] = 0.02

        p.addUserDebugLine(a1, c1, [1,1,1], 7)
        p.addUserDebugLine(b1, d1, [1,1,1], 7)

        if shouldParkedCarsBeLoaded:
            if self.left_only is not True:
                for i in range (8):
                    if i == b:
                        continue
                    if self.shouldParkedCarsBeMovedBackAndForth:
                        vel = random.randint(2, 5)
                        force = random.randint(50, 120)
                        self.right_cars[i] = Car(self.client, basePosition=[6.2, 0.8+1.5*i, 0.1], val=vel, maxForce=force)
                    else:
                        self.right_cars[i] = p.loadURDF("husky/husky.urdf", basePosition=[6.2, 0.8+1.5*i, 0.1], useFixedBase = True)
                    p.stepSimulation()
                    time.sleep(1/240)

            a2 = [5.6, 0.2+1.5*b, 0]
            b2 = [5.6, 0.2+1.5*(b+1), 0]
            c2 = [7.2, 0.2+1.5*b, 0]
            d2 = [7.2, 0.2+1.5*(b+1), 0]
            self.slot2 = np.array([a2[:2], b2[:2], c2[:2], d2[:2]])

            b2[2] = 0.02
            c2[2] = 0.02
            a2[2] = 0.02
            d2[2] = 0.02

            p.addUserDebugLine(a2, c2, [1,1,1], 7)
            p.addUserDebugLine(b2, d2, [1,1,1], 7)

        if slot_side != 'right':
            self.parking_location = self.slot1
        else:
            self.parking_location = self.slot2

        p.stepSimulation(self.client)

    def __moveCarsRandomly(self):
        print(self.left_cars)
        for idx, car in enumerate(self.left_cars):
            if car is None:
                continue
            pos = car.get_observation()[0]
            if pos[0] > 2:
                self.la[idx] = 2
            if pos[0] < 0.9:
                self.la[idx] = 1
            car.apply_action(self.la[idx], discrete = True)
        if self.left_only is not True:
            for idx, car in enumerate(self.right_cars):
                if car is None:
                    continue
                pos = car.get_observation()[0]
                if pos[0] > 6.3:
                    self.ra[idx] = 2
                if pos[0] < 5.5:
                    self.ra[idx] = 1
                car.apply_action(self.ra[idx], discrete = True)

    def step(self, action):
        """
        Takes one step.

        Arguments:\n
            - action -- A list of the form [throttle, steering_angle]

        Return:\n
            - obs -- The observation of the next state
            - reward -- The reward for the current time step
            - done -- True if the episode is complete, else False
            - info -- Additional Information
        """
        # Feed action to the car and get observation of car's state
        self.t.apply_action(action)
        if self.shouldParkedCarsBeMovedBackAndForth:
            self.__moveCarsRandomly()
        
        # steering = action
        # print('steering:', steering)
        p.stepSimulation()
        car_ob, self.vector = self.t.get_observation()

        pos = np.array(car_ob[:2])
        # ori = np.array(car_ob[2:4])
        # vel = np.array(car_ob[4:])

        slot_center = np.sum(self.parking_location, axis = 0) / 4

        # print('pos:', pos)
        # print('ori:', ori)
        # print('vel:', vel)


        # slot_vector = 0
        target_vector = math.pi

        # print('car_ori:', self.vector)
        # print('slot_vector:', slot_vector)
        
        # print()
        # reward function considering feed from distance measuring sensors and computing distance from goal for each agent
        reward = 0
        reward += np.sum(abs(pos - slot_center)) * -5
        # TODO: See if the reward procedure can be divided into 2 parts. One for reaching the centre, and the other for parking
        # reward += abs(self.vector - target_vector) * -1

        # contact_points = p.getContactPoints(self.t.get_ids()[0])

        # print('contact_points:', contact_points)



        # Condition for done
        self.done = False
        if np.sum(abs(pos - slot_center)) < 0.8: #and abs(self.vector - target_vector) < 0.1:
            reward = 500
            self.done = True
            # print("\nPark Ho Gyi\n")
            # print("Co-ordinates while parking:")
            # print('\tslot_center', slot_center)
            # print('\tpos', pos)

        dist = self.__getDistances(self.car)
        # print("dist----------------------------",dist)

        ob = (list(car_ob) + dist)
        dist.sort()
        if(dist[0] < 0.28):
            self.done = True
            # print("\nYe Kya Hua\n")
            # print('Co-ordinates after collision')
            # print('\tpos', pos)
            # print('\tsensor distances:', dist)
            reward -= 300
        
        # print(car_ob)
        # print('\nreward:', reward)
        # print("observ---",ob)
       
        return ob, reward, self.done, dict()

    def seed(self, seed=None):
        """Getting the seed of the environment"""
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def close(self):
        """Shuts down the simulation"""
        p.disconnect(self.client)


class Car:
    def __init__(self, client, basePosition = [3.7,0.2, 0.2], baseOrientationEuler = None,
    c_throttle = 20, c_drag = 0.01, c_rolling = 0.2, joint_speed = 0.1, val = 3, maxForce = 100,
    carType = 'husky'):
        self.client = client
        #f_name = os.path.join(os.path.dirname(__file__), 'simplecar.urdf')
        urdfname = carType + '/' + carType + '.urdf'
        if baseOrientationEuler is None:
            self.car = p.loadURDF(fileName=urdfname,
                              basePosition = basePosition)
        else:
            self.car = p.loadURDF(fileName=urdfname,
                              basePosition = basePosition,
                              baseOrientation=p.getQuaternionFromEuler(baseOrientationEuler))
        # print(self.car)
        # print("its running")
        # Joint indices as found by p.getJointInfo()
        if carType == 'husky':
            self.steering_joints = [0, 2]
            self.drive_joints = [1, 3, 4, 5]
        else:
            self.steering_joints = [4, 6]
            self.drive_joints = [2, 3, 7, 5]
        # Joint speed
        self.joint_speed = joint_speed
        # Drag constants
        self.c_rolling = c_rolling
        self.c_drag = c_drag
        # Throttle constant increases "speed" of the car
        self.c_throttle = c_throttle

        self.val = val
        self.maxForce = maxForce

    def get_ids(self):
        return self.car, self.client

    def apply_action(self, action, discrete = False):
        if discrete:
            # For discrete actions on husky
            # Expects action to be two dimensional
            a = action
            val = self.val #rad/s
            maxForce = self.maxForce #Newton

            if(a==0): #right
                targetVel=val
                for joint in range(2, 6):
                    targetVel = 3
                    for joint in range(1,3):
                        p.setJointMotorControl2(self.car,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                    for joint in range(1,3):
                        p.setJointMotorControl2(self.car,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)
                    p.stepSimulation()
            if(a==1): #forward
                targetVel = val
                for joint in range(2, 6):
                    p.setJointMotorControl2(self.car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                p.stepSimulation()
            if(a==2): #back
                targetVel = -val
                for joint in range(2, 6):
                    p.setJointMotorControl2(self.car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                p.stepSimulation()
            if(a==3): #left
                targetVel = 3
                for joint in range(1,3):
                    p.setJointMotorControl2(self.car,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(self.car,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1* targetVel,force = maxForce)
                p.stepSimulation()
            if(a==4): #stop
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(self.car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                p.stepSimulation()

        else:

            throttle, steering_angle = action

            # Clip throttle and steering angle to reasonable values
            throttle = min(max(throttle, 0), 1)
            steering_angle = max(min(steering_angle, 0.5), -0.5)

            # Set the steering joint positions
            p.setJointMotorControlArray(self.car, self.steering_joints,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=[steering_angle] * 2,
                                        physicsClientId=self.client)

            # Calculate drag / mechanical resistance ourselves
            # Using velocity control, as torque control requires precise models
            # friction = -self.joint_speed * (self.joint_speed * self.c_drag +
            #                                 self.c_rolling)
            # acceleration = self.c_throttle * throttle + friction
            # Each time step is 1/240 of a second
            # self.joint_speed = self.joint_speed + 1/30 * acceleration
            # if self.joint_speed < 0:
            #     self.joint_speed = 0

            # Set the velocity of the wheel joints directly
            p.setJointMotorControlArray(bodyUniqueId=self.car,
                                        jointIndices=self.drive_joints,
                                        controlMode=p.VELOCITY_CONTROL,
                                        targetVelocities=[30*throttle] * 4,
                                        forces=[20] * 4,
                                        physicsClientId=self.client)
            # throttle, steering_angle = action

            # Clip throttle and steering angle to reasonable values
            # throttle = min(max(throttle, 0), 1)
            # steering_angle = max(min(steering_angle,0.5), -0.5)

            # # Set the steering joint positions
            # p.setJointMotorControlArray(self.car, self.steering_joints,
            #                             controlMode=p.POSITION_CONTROL,
            #                             targetPositions=[steering_angle] * 2,
            #                             physicsClientId=self.client)

            # # Calculate drag / mechanical resistance ourselves
            # # Using velocity control, as torque control requires precise models
            # friction = -self.joint_speed * (self.joint_speed * self.c_drag +
            #                                 self.c_rolling)
            # acceleration = self.c_throttle * throttle + friction
            # # Each time step is 1/240 of a second
            # self.joint_speed = self.joint_speed + 1/30 * acceleration
            # if self.joint_speed < 0:
            #     self.joint_speed = 0

            # # Set the velocity of the wheel joints directly
            # p.setJointMotorControlArray(bodyUniqueId=self.car,
            #                             jointIndices=self.drive_joints,
            #                             controlMode=p.VELOCITY_CONTROL,
            #                             targetVelocities=[self.joint_speed] * 4,
            #                             forces=[10] * 4,
            #                             physicsClientId=self.client)

    def get_observation(self):
        # Get the position and orientation of the car in the simulation
        pos, ang = p.getBasePositionAndOrientation(self.car, self.client)
        ang = p.getEulerFromQuaternion(ang)
        
        ori = (math.cos(ang[2]), math.sin(ang[2]))
        ori = list(ori)
        ori = [round(x,5) for x in ori]
        pos = pos[:2]
        pos = list(pos)
        # print("pos----",pos)
        pos=[round(x,5) for x in pos]
        # Get the velocity of the car
        vel = p.getBaseVelocity(self.car, self.client)[0][0:2]
        vel = list(vel)
        vel = [round(x,5) for x in vel]
        vector = round(ang[2],4)
        # Concatenate position, orientation, velocity
        observation = (pos + ori + vel)

        # print("vel;;;;",vector)
        # print("pos;;;",pos)
        # print("ang[2];;;",ang[2])
        return observation, vector
