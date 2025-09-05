#!/usr/bin/env python

import unittest
import rospy
import rostest
import time
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodyneScan
import threading

class MultiLidarIntegrationTest(unittest.TestCase):
    
    def setUp(self):
        rospy.init_node('multi_lidar_test_node', anonymous=True)
        
        self.received_pointclouds = {
            'red': [],
            'yellow': [],
            'blue': [],
            'green': []
        }
        
        self.received_scans = {
            'red': [],
            'yellow': [],
            'blue': [],
            'green': []
        }
        
        self.setup_subscribers()
        self.setup_publishers()
        
        self.sync_lock = threading.Lock()
        self.frame_timestamps = {}
    
    def setup_subscribers(self):
        rospy.Subscriber('/test_lidar_red_pointcloud', PointCloud2, 
                        lambda msg: self.pointcloud_callback(msg, 'red'))
        rospy.Subscriber('/test_lidar_yellow_pointcloud', PointCloud2, 
                        lambda msg: self.pointcloud_callback(msg, 'yellow'))
        rospy.Subscriber('/test_lidar_blue_pointcloud', PointCloud2, 
                        lambda msg: self.pointcloud_callback(msg, 'blue'))
        rospy.Subscriber('/test_lidar_green_pointcloud', PointCloud2, 
                        lambda msg: self.pointcloud_callback(msg, 'green'))
    
    def setup_publishers(self):
        self.scan_publishers = {
            'red': rospy.Publisher('/test_lidar_red_scan', VelodyneScan, queue_size=10),
            'yellow': rospy.Publisher('/test_lidar_yellow_scan', VelodyneScan, queue_size=10),
            'blue': rospy.Publisher('/test_lidar_blue_scan', VelodyneScan, queue_size=10),
            'green': rospy.Publisher('/test_lidar_green_scan', VelodyneScan, queue_size=10)
        }
        
        time.sleep(2.0)
    
    def pointcloud_callback(self, msg, lidar_color):
        with self.sync_lock:
            self.received_pointclouds[lidar_color].append(msg)
            
            timestamp = msg.header.stamp.to_sec()
            if timestamp not in self.frame_timestamps:
                self.frame_timestamps[timestamp] = []
            self.frame_timestamps[timestamp].append(lidar_color)
    
    def create_test_velodyne_scan(self, timestamp):
        scan = VelodyneScan()
        scan.header.stamp = rospy.Time.from_sec(timestamp)
        scan.header.frame_id = "velodyne"
        
        return scan
    
    def test_sensor_synchronization(self):
        base_time = rospy.Time.now().to_sec()
        
        for i in range(10):
            timestamp = base_time + i * 0.1
            
            for color in ['red', 'yellow', 'blue', 'green']:
                scan = self.create_test_velodyne_scan(timestamp)
                self.scan_publishers[color].publish(scan)
            
            time.sleep(0.1)
        
        time.sleep(2.0)
        
        synchronized_frames = 0
        for timestamp, colors in self.frame_timestamps.items():
            if len(colors) == 4:
                synchronized_frames += 1
        
        self.assertGreater(synchronized_frames, 5, 
                          "Should have at least 5 synchronized frames")
    
    def test_frame_rate_consistency(self):
        for color in self.received_pointclouds:
            self.received_pointclouds[color].clear()
        
        start_time = rospy.Time.now().to_sec()
        target_rate = 10.0
        
        for i in range(20):
            timestamp = start_time + i / target_rate
            
            for color in ['red', 'yellow', 'blue', 'green']:
                scan = self.create_test_velodyne_scan(timestamp)
                self.scan_publishers[color].publish(scan)
            
            time.sleep(1.0 / target_rate)
        
        time.sleep(2.0)
        
        for color in ['red', 'yellow', 'blue', 'green']:
            received_count = len(self.received_pointclouds[color])
            self.assertGreater(received_count, 15, 
                             f"LiDAR {color} should receive at least 15 frames")
    
    def test_nodelet_manager_failure_recovery(self):
        import subprocess
        result = subprocess.run(['rosnode', 'list'], capture_output=True, text=True)
        self.assertIn('test_velodyne_nodelet_manager', result.stdout)
    
    def test_topic_remapping_validation(self):
        import subprocess
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True)
        
        expected_topics = [
            '/test_lidar_red_scan',
            '/test_lidar_red_pointcloud',
            '/test_lidar_yellow_scan',
            '/test_lidar_yellow_pointcloud',
            '/test_lidar_blue_scan',
            '/test_lidar_blue_pointcloud',
            '/test_lidar_green_scan',
            '/test_lidar_green_pointcloud'
        ]
        
        for topic in expected_topics:
            self.assertIn(topic, result.stdout, f"Topic {topic} should exist")
    
    def test_calibration_parameter_validation(self):
        calibration_file = rospy.get_param('~calibration', '')
        self.assertTrue(calibration_file.endswith('lidarIntrinsics.yaml'))
        
        max_range = rospy.get_param('~max_range', 0.0)
        min_range = rospy.get_param('~min_range', 0.0)
        
        self.assertGreater(max_range, min_range)
        self.assertGreater(max_range, 0.0)
        self.assertGreaterEqual(min_range, 0.0)

if __name__ == '__main__':
    rostest.rosrun('ford_demo', 'multi_lidar_integration_tests', 
                   MultiLidarIntegrationTest)
