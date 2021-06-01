#!/usr/bin/env python

"""
Particle filter localization implementation
"""

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, Point32
from nav_msgs.srv import GetMap
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from tf import TransformListener, TransformBroadcaster, transformations
from math import pi, fabs
import numpy as np
from numpy.random import random_sample
from sklearn.svm import OneClassSVM
from obstacle_map import ObstacleMap
from help_funcs import invert_pose_tf, trans_rot_to_pose, pose_to_triple, angle_diff, get_rot_mat


class Particle:
    """ 
    The Particle class employed in creating a hypothesis (particle) of 
    the robot's pose consisting of x, y, theta (yaw) and w (weight).
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, odom_correct=1.45, w=1.0):
        """ 
        Particle constructor
        """

        self.x = x  # Particle x-coordinate relative to the map frame
        self.y = y  # Particle y-coordinate relative to the map frame
        self.w = w  # Particle weight
        self.theta = theta  # Particle theta angle (yaw) relative to the map frame
        self.odom_correct = odom_correct  # Multiplier to correct turning measurements of the odometry

    def conv_to_pose(self):
        """
        Convert a particle to a geometry_msgs/Pose message
        """
        orient_tpl = transformations.quaternion_from_euler(0, 0, self.theta)
        pose_msg = Pose(position=Point(x=self.x, y=self.y, z=0),
                        orientation=Quaternion(x=orient_tpl[0],
                                               y=orient_tpl[1],
                                               z=orient_tpl[2],
                                               w=orient_tpl[3]))
        return pose_msg

    def init_on_map(self, grid_map):
        """
        Initialize the current particle to be at some random location and 
        with a random orientation on the provided map.
        """

        min_x = grid_map.info.origin.position.x
        min_y = grid_map.info.origin.position.y
        max_x = min_x + grid_map.info.width * grid_map.info.resolution
        max_y = min_y + grid_map.info.height * grid_map.info.resolution
        # min_x = 0
        # min_y = -1
        # max_x = 5
        # max_y = 4

        ptcl_x = float(np.random.uniform(min_x, max_x))
        ptcl_y = float(np.random.uniform(min_y, max_y))

        theta = float(np.random.uniform(-np.pi, np.pi))

        self.__init__(ptcl_x, ptcl_y, theta)
        return self


class ParticleFilterLoc:
    """ 
    Particle filter (PF) localization class
    """

    def __init__(self):

        self.init_done = False  # Boolean flag to enable localization only after particle filter initialization is completed
        rospy.init_node('pfl')  # Create the particle filter roscore node

        self.base_frame = "base_link"  # Robot base frame
        self.map_frame = "map"  # Map coordinate frame
        self.odom_frame = "odom"  # Odometry coordinate frame
        self.ls_topic = "base_scan"  # Laser scan topic
        self.ptcl_num = 250  # Number of localization particles of the filter
        self.lin_th = 0.15  # The amount of linear movement needed before updating (linear threshold)
        self.rot_th = pi / 6  # The amount of angular movement needed before updating (rotation threshold)
        self.lost_prob = .4  # The probability given to the robot being "lost" at any given time
        self.outl_num = int(self.ptcl_num * self.lost_prob * 0.5)  # The number of outliers to keep around

        # Subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        self.pose_sub = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.init_pose_update)

        # Particle cloud and particle filter marker publishers for rviz visualization
        self.ptcl_pub = rospy.Publisher("ptcl_cloud", PoseArray, queue_size=10)

        # LaserScan (Lidar) subscriber
        self.ls_sub = rospy.Subscriber(self.ls_topic, LaserScan, self.laser_cb)

        # Coordinate transform listener and broadcaster attributes
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.ptcl_cloud = []  # Particle cloud list (probability distribution of vacuum cleaner (VC) poses)

        # Tuple to hold the VC pose in the odometry frame when the last filter update was performed.
        # Format = (x,y,theta) (where theta is the yaw)
        self.curr_odom_triple = []

        # Request the map (type = nav_msgs/OccupancyGrid) from the map server
        # where the localization will occur
        rospy.wait_for_service('/static_map')
        self.map_server = rospy.ServiceProxy('/static_map', GetMap)
        self.map = self.map_server().map

        self.obst_map = ObstacleMap(self.map)
        self.init_done = True

    def rob_pos_update(self):
        """ 
        Update the estimate of the robot's pose given the updated particles.
        """

        self.norm_ptcl_weights()  # Normalize the particle weights
        max_weight = max(self.ptcl_cloud, key=lambda p: p.w)  # Get the highest particle weight
        self.rob_pose = max_weight.conv_to_pose()  # Assign the robot's pose to the highest weight particle pose

    def odom_ptcl_update(self):
        """ 
        Update the particles using the newly given odometry pose.
        The function calculates the delta (dt) value which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """

        new_odom_triple = pose_to_triple(self.odom_pose.pose)

        # Calculate the (x, y, theta) triple difference of the last odometry update
        if self.curr_odom_triple:
            prev_odom_triple = self.curr_odom_triple
            dt = (new_odom_triple[0] - self.curr_odom_triple[0],
                  new_odom_triple[1] - self.curr_odom_triple[1],
                  angle_diff(new_odom_triple[2], self.curr_odom_triple[2]))

            self.curr_odom_triple = new_odom_triple
        else:
            self.curr_odom_triple = new_odom_triple
            return

        # For each particle cloud particle
        for ptcl in self.ptcl_cloud:
            # Calculate the angle difference between the old odometry position
            # and the old particle position. Then create a rotation matrix between
            # the two angles
            rot_mat = get_rot_mat(ptcl.theta - prev_odom_triple[2])

            # Rotate the robot motion vector and apply the result to the current particle
            rotated_dt = np.dot(rot_mat, dt[:2])

            # Include randomness
            lin_rand = np.random.normal(1, 0.2)
            ang_rand = np.random.uniform(ptcl.odom_correct, 0.3)

            ptcl.x += rotated_dt[0] * lin_rand
            ptcl.y += rotated_dt[1] * lin_rand

            ptcl.theta += dt[2] * ang_rand
            ptcl.theta = angle_diff(ptcl.theta, 0)

    def get_inl_outl(self):
        """ 
        Utilize Scikit Learn OneClassSVM in choosing particles to 'ommit' 
        via unsupervised outlier detection. Return inlier and outlier lists.
        """

        # Format train data
        x = [p.x for p in self.ptcl_cloud]
        y = [p.y for p in self.ptcl_cloud]
        x_train = np.column_stack((x, y))

        # Create an unsupervised outlier detection model
        # The lower the 'nu' variable the fewer outliers picked
        clf = OneClassSVM(nu=.3, kernel="rbf", gamma=0.1)
        clf.fit(x_train)  # Detect the soft boundary of the set of samples

        # Predict inliers and outliers
        y_pred_train = clf.predict(x_train)

        # Create inlier and outlier particle lists
        inls = []  # Particles to remain
        outls = []  # Particles to ommit

        # Loop through the particle cloud and predictions and populate lists accordingly
        for ptcl, pred in zip(self.ptcl_cloud, y_pred_train):
            if pred == 1:
                inls.append(ptcl)
            elif pred == -1:
                outls.append(ptcl)

        return inls, outls

    def resample(self):
        """ 
        Resample the particles according to the new particle weights.
        The weights determine the probability of selecting a particular
        particle when resampling.
        """

        self.norm_ptcl_weights()  # Normalize the particle weights

        # Calculate inlaying and exploring particle sets
        inls, outls = self.get_inl_outl()
        outls_needed = int(self.ptcl_num * self.lost_prob)
        inls_needed = int(self.ptcl_num - outls_needed)

        # Calculate the average odom_correct of the inliers
        corr_mean = np.mean([ptcl.odom_correct for ptcl in inls])

        # Get the probability weights of the particles in the particle cloud
        # and use them to extract random inlier sample
        probs = [ptcl.w for ptcl in self.ptcl_cloud]
        new_inls = self.get_rand_sample(self.ptcl_cloud, probs, inls_needed)

        # Recalculate outliers
        # This keeps some number of outlying particles around unchanged, and spreads the rest randomly around the map.
        if outls_needed > min(len(outls), self.outl_num):
            outls.sort(key=lambda p: p.w, reverse=True)

            num_to_make = outls_needed - min(len(outls), self.outl_num)

            new_outls = outls[:self.outl_num] + [Particle().init_on_map(self.map) for _ in range(num_to_make)]
            for p in new_outls:
                p.odom_correct = corr_mean
        else:
            new_outls = outls[:outls_needed]

        new_ptcls = new_inls + new_outls  # All ptcls are inliers + outliers

        # Set all of the weights back to the same value. 
        # Concentration of particles now reflects weight.
        for ptcl in new_ptcls:
            ptcl.w = 1.0
            ptcl.odom_correct = np.random.normal(ptcl.odom_correct, 0.1)

        self.norm_ptcl_weights()
        self.ptcl_cloud = new_ptcls  # Set the particle cloud to the new particle list

    @staticmethod
    def ls_pt_prob(dist):
        """
        Return the probability of the laser returning the point at a given 
        distance from the obstacle.
        """

        k = 0.1  # meters of half-life of distance probability for main distribution
        ls_noise = 0.05  # Laser scan noise
        prob = (1 / (1 + ls_noise)) * (ls_noise + 1 / (abs(dist) / k + 1))

        return prob

    def ls_ptcl_update(self, ls_data):
        """
        Update the particle weights based on the received laser scan data
        """

        # Create a point cloud (transform to cartesian coordinates)
        ls_pts = PointCloud()
        ls_pts.header = ls_data.header

        for idx, rng in enumerate(ls_data.ranges):
            if rng == 0:
                continue

            # Point to laser scan coordinate frame
            angle = ls_data.angle_min + idx * ls_data.angle_increment
            x = rng * np.cos(angle)
            y = rng * np.sin(angle)
            ls_pts.points.append(Point32(x=x, y=y))

        # Transform the point cloud into 'base_link' frame coordinates and assign
        ls_pts = self.tf_listener.transformPointCloud('base_link', ls_pts)

        # Create a 3x3 matrix for each particle for point transformation
        for ptcl in self.ptcl_cloud:
            rot_mat = np.matrix([[np.cos(ptcl.theta), -np.sin(ptcl.theta), 0],
                                 [np.sin(ptcl.theta), np.cos(ptcl.theta), 0],
                                 [0, 0, 1]])

            trans_mat = np.matrix([[1, 0, ptcl.x],
                                   [0, 1, ptcl.y],
                                   [0, 0, 1]])

            mtx = np.dot(trans_mat, rot_mat)

            # For each laser scan point adjust it onto the current particle
            probs = []
            for pt in ls_pts.points:
                xy = np.dot(mtx, np.array([pt.x, pt.y, 1]))

                # Get the distance to the closest obstacle
                obstacle_dist = self.obst_map.get_dist_to_obstacle(xy.item(0), xy.item(1))
                if np.isnan(obstacle_dist):
                    continue

                probs.append(self.ls_pt_prob(obstacle_dist))

            # Sum the entire laser scan probabilities and normalize
            tot_prob = np.sum([p ** 3 for p in probs]) / len(probs)

            # Update the particle's probability given the total probability
            ptcl.w *= tot_prob

        self.norm_ptcl_weights()  # Normalize particles

    @staticmethod
    def get_rand_sample(choices, probs, size):
        """
        Return a random sample of the given size from the set choices 
        with the specified probabilities.
        """

        values = np.array(range(len(choices)))  # Array of values to sample from
        probs = np.array(probs)  # Numpy array of probabilities of selecting 'choices' elements
        bins = np.add.accumulate(probs)
        # arr = np.digitize(random_sample(size), bins)

        # np.digitize returns IndexError at times 
        inds = values[np.digitize(random_sample(size), bins[:-1])]  # values[np.histogram(random_sample(size), bins)[0]]

        # Populate the sample list to return ('choices' are deepcopied so that 
        # the changes are not reflected when not needed)
        sample = []
        for i in inds:
            sample.append(deepcopy(choices[int(i)]))
        return sample

    def init_pose_update(self, msg):
        """ 
        Reinitialize the particle filter cloud based on a pose estimate 
        (rviz GUI).
        """

        triple = pose_to_triple(msg.pose.pose)
        self.init_ptcls(triple)
        self.base_odom_update(msg)

    def init_ptcls(self, triple=None):
        """
        Initialize the particle cloud
        """

        # (x, y, theta (yaw)) triple employed in particle cloud initialization.
        # If None the robot odometry pose will be assigned (used).
        if triple is None:
            triple = pose_to_triple(self.odom_pose.pose)
        self.ptcl_cloud = []

        lin_var, ang_var = 0.5, 4  # Linear and angular variances in meters

        # Initial particle (x, y) coordinate and orientation (theta) lists
        x_coords = np.random.normal(triple[0], lin_var, size=self.ptcl_num)
        y_coords = np.random.normal(triple[1], lin_var, size=self.ptcl_num)
        thetas = np.random.vonmises(triple[2], ang_var, size=self.ptcl_num)

        self.ptcl_cloud = [Particle(x=x_coords[i], y=y_coords[i], theta=thetas[i]) for i in range(self.ptcl_num)]

        self.norm_ptcl_weights()  # Normalize the particle cloud weights
        self.rob_pos_update()  # Update the robot pose

    def norm_ptcl_weights(self):
        """
        Normalize every particle weight with the total weight
        """

        tot_w = sum([p.w for p in self.ptcl_cloud])  # Total weight of all particles

        if tot_w != 0:
            for p in self.ptcl_cloud:
                p.w /= tot_w

    def pub_ptcls(self, msg):
        """
        Visualize particle poses and markers in rviz
        """

        ptcl_poses = []
        for p in self.ptcl_cloud:
            ptcl_poses.append(p.conv_to_pose())

        # Publish particle poses so they can be viewed in rviz
        self.ptcl_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                      frame_id=self.map_frame),
                                        poses=ptcl_poses))

    def laser_cb(self, ls_data):
        """ 
        Process laser scan data (type = sensor_msgs/LaserScan)
        """

        # If initialization is not complete or the transform b/w the 
        # base frame and laser scan or odometry data frames is not possible
        # then return and wait
        if not self.init_done or \
                not (self.tf_listener.canTransform(self.base_frame, ls_data.header.frame_id, ls_data.header.stamp)) or \
                not (self.tf_listener.canTransform(self.base_frame, self.odom_frame,
                                                   ls_data.header.stamp)):  # Return if initialization is not complete
            return

        # Get the laser pose relative to the robot base frame
        p = PoseStamped(header=Header(stamp=rospy.Time(0), frame_id=ls_data.header.frame_id))
        self.ls_pose = self.tf_listener.transformPose(self.base_frame, p)

        # Update the robot belief pose based on its odometry readings
        p = PoseStamped(header=Header(stamp=ls_data.header.stamp, frame_id=self.base_frame), pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        new_odom_triple = pose_to_triple(self.odom_pose.pose)  # Assign the odometry pose in (x,y,theta) format 

        if not self.ptcl_cloud:
            self.init_ptcls()  # If the particle cloud has not been initialized yet then do it
            self.curr_odom_triple = new_odom_triple  # In this case the new odometry pose equals the current one
            self.base_odom_update(ls_data)  # Update frame transforms

        # Update only if we have passed the linear movement or rotation thresholds (moved or rotated enough)
        elif (fabs(new_odom_triple[0] - self.curr_odom_triple[0]) > self.lin_th or
              fabs(new_odom_triple[1] - self.curr_odom_triple[1]) > self.lin_th or
              fabs(new_odom_triple[2] - self.curr_odom_triple[2]) > self.rot_th):

            self.odom_ptcl_update()  # Update based on odometry data
            self.ls_ptcl_update(ls_data)  # Update based on laser scan
            self.rob_pos_update()  # Update robot's pose
            self.resample()  # Resample particles to emphasize on high density areas

        self.pub_ptcls(ls_data)  # Publish particles so they can be visualized in rviz

    def base_odom_update(self, data):
        """ 
        Update the offset of the map and odometry frames based on the latest localizer data.
        """

        transl, rot = invert_pose_tf(self.rob_pose)
        p = PoseStamped(pose=trans_rot_to_pose(transl, rot),
                        header=Header(stamp=data.header.stamp, frame_id=self.base_frame))
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(1.0))
        self.tf_listener.lookupTransform(self.base_frame, self.odom_frame, data.header.stamp)
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        self.transl, self.rot = invert_pose_tf(self.odom_to_map.pose)

    def bc_trfm(self):
        """ 
        Ensure last odometry to map transformation is broadcasted. 
        """

        # Return if translation and rotation attributes are not assigned
        if not (hasattr(self, 'transl') and hasattr(self, 'rot')):
            return

        self.tf_broadcaster.sendTransform(self.transl,
                                          self.rot,
                                          rospy.get_rostime(),
                                          self.odom_frame,
                                          self.map_frame)


if __name__ == '__main__':
    pfl = ParticleFilterLoc()  # Initialize the particle filter localization
    r = rospy.Rate(5)

    # Repeatedly broadcast the latest odom to map transform
    while not rospy.is_shutdown():
        pfl.bc_trfm()
        r.sleep()
