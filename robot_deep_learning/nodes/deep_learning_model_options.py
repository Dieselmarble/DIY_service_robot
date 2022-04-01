#!/usr/bin/env python3
import os

def get_directory():
    return os.environ['HELLO_FLEET_PATH'] + '/deep_perception_models/'

def get_face_directory():
    return os.environ['HELLO_FLEET_PATH'] + '/faces/'

def use_neural_compute_stick():
    # Currently this only work for two models: object and face
    # detection.  Currently it does not work with head pose
    # estimation, facial landmarks, and body landmarks.
    return False
