#!/usr/bin/env python3

import os
import glob
import cv2
import sys
import rospy
import numpy as np
import face_reidentification as fr
import reidentification_node_python3 as ri
import deep_learning_model_options as do

if __name__ == '__main__':    
    print('cv2.__version__ =', cv2.__version__)
    print('Python version (must be > 3.0):', sys.version)
    assert(int(sys.version[0]) >= 3)
    # Filtering for depths corresponding with heads with heights
    # or widths from 8cm to 40cm should be conservative.
    min_head_m = 0.08
    max_head_m = 0.4

    # find model directory where .xml and .bin is saved
    models_directory = do.get_directory()
    faces_directory = do.get_face_directory
    print('Using the following directory for deep learning models:', models_directory)        
    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')

    identifier = fr.FaceReidentificator(models_directory, faces_directory,
                        use_neural_compute_stick=use_neural_compute_stick)

    default_marker_name = 'faceID'
    node_name = 'FaceIDNode'
    topic_base_name = 'faceID'
    fit_plane = False
    node = ri.IdentificationNode(identifier,
                                default_marker_name,
                                node_name,
                                topic_base_name,
                                fit_plane,
                                min_box_side_m=min_head_m,
                                max_box_side_m=max_head_m)
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')