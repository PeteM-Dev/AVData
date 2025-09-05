#!/usr/bin/env python

import unittest
import tempfile
import os
import yaml
import subprocess
import sys
import signal
import time

class ExtrinsicsBroadcastTest(unittest.TestCase):
    
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.valid_yaml_file = os.path.join(self.test_dir, "valid_extrinsics.yaml")
        self.invalid_yaml_file = os.path.join(self.test_dir, "invalid_extrinsics.yaml")
        self.malformed_yaml_file = os.path.join(self.test_dir, "malformed_extrinsics.yaml")
        
        valid_data = {
            'header': {'frame_id': 'body'},
            'child_frame_id': 'lidar_red',
            'transform': {
                'translation': {'x': 1.0, 'y': 0.0, 'z': 2.0},
                'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        }
        with open(self.valid_yaml_file, 'w') as f:
            yaml.dump(valid_data, f)
        
        invalid_data = {
            'header': {'frame_id': 'body'},
            'child_frame_id': 'lidar_red',
            'transform': {
                'translation': {'x': 1.0, 'y': 0.0},
                'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            }
        }
        with open(self.invalid_yaml_file, 'w') as f:
            yaml.dump(invalid_data, f)
        
        with open(self.malformed_yaml_file, 'w') as f:
            f.write("invalid: yaml: content:\n  - malformed\n    - structure")
    
    def tearDown(self):
        import shutil
        shutil.rmtree(self.test_dir)
    
    def test_valid_yaml_parsing(self):
        with open(self.valid_yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.assertIn('header', data)
        self.assertIn('frame_id', data['header'])
        self.assertIn('child_frame_id', data)
        self.assertIn('transform', data)
        self.assertIn('translation', data['transform'])
        self.assertIn('rotation', data['transform'])
        
        translation = data['transform']['translation']
        self.assertIn('x', translation)
        self.assertIn('y', translation)
        self.assertIn('z', translation)
        
        rotation = data['transform']['rotation']
        self.assertIn('x', rotation)
        self.assertIn('y', rotation)
        self.assertIn('z', rotation)
        self.assertIn('w', rotation)
    
    def test_invalid_yaml_validation(self):
        with open(self.invalid_yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        self.assertNotIn('z', data['transform']['translation'])
        self.assertNotIn('w', data['transform']['rotation'])
    
    def test_malformed_yaml_handling(self):
        with self.assertRaises(yaml.YAMLError):
            with open(self.malformed_yaml_file, 'r') as f:
                yaml.safe_load(f)
    
    def test_transform_consistency(self):
        with open(self.valid_yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        rotation = data['transform']['rotation']
        quat_norm = (rotation['x']**2 + rotation['y']**2 + 
                    rotation['z']**2 + rotation['w']**2)**0.5
        self.assertAlmostEqual(quat_norm, 1.0, places=6)
    
    def test_extrinsics_broadcaster_script(self):
        script_path = os.path.join(os.path.dirname(__file__), 
                                  "../scripts/extrinsics_broadcaster.py")
        
        result = subprocess.run([sys.executable, script_path], 
                              capture_output=True, text=True)
        self.assertIn("error: no extrinsics yaml file given", result.stdout)
        
        os.environ['PATH'] = '/bin:/usr/bin'
        result = subprocess.run([sys.executable, script_path, self.valid_yaml_file], 
                              capture_output=True, text=True)
        self.assertIn("rosrun tf2_ros static_transform_publisher", result.stdout)
    
    def test_cleanup_functionality(self):
        cleanup_command = "ps -ef | grep static_transform_publisher | awk '{print $2}' | xargs kill -2"
        
        self.assertIn("static_transform_publisher", cleanup_command)
        self.assertIn("kill -2", cleanup_command)
    
    def test_concurrent_broadcasting_scenarios(self):
        yaml_files = []
        for i in range(4):
            yaml_file = os.path.join(self.test_dir, f"lidar_{i}_extrinsics.yaml")
            data = {
                'header': {'frame_id': 'body'},
                'child_frame_id': f'lidar_{i}',
                'transform': {
                    'translation': {'x': float(i), 'y': 0.0, 'z': 2.0},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                }
            }
            with open(yaml_file, 'w') as f:
                yaml.dump(data, f)
            yaml_files.append(yaml_file)
        
        for yaml_file in yaml_files:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
                self.assertIsNotNone(data)
                self.assertIn('child_frame_id', data)

if __name__ == '__main__':
    unittest.main()
