#!/usr/bin/env python3

import os
import glob
import cv2
import sys
import rospy
import numpy as np
import face_reidentification as fr
import detection_node_python3 as dn
import deep_learning_model_options as do
import head_estimator_python3 as he
from argparse import ArgumentParser, SUPPRESS

debug_output = True


def build_argparser():
    parser = ArgumentParser(add_help=False)
    args = parser.add_argument_group('Options')
    args.add_argument('-h', '--help', action='help', default=SUPPRESS,
                        help='Show this help message and exit.')
    args.add_argument('-i', '--input', 
                        help='Required. Path to ground truth image folder', required=True, type=str)
    args.add_argument('-o', '--output', 
                    help='Required. Path to save the embedded features', required=True, type=str)
    
    return parser

if __name__ == '__main__':    
    args = build_argparser().parse_args()
    print('cv2.__version__ =', cv2.__version__)
    print('Python version (must be > 3.0):', sys.version)
    assert(int(sys.version[0]) >= 3)
    # Filtering for depths corresponding with heads with heights
    # or widths from 8cm to 40cm should be conservative.
    min_head_m = 0.08
    max_head_m = 0.4

    # find model directory where .xml and .bin is saved
    models_directory = do.get_directory()
    faces_directory = args.input
    print('Using the following directory for deep learning models:', models_directory)        
    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')

    detector = he.HeadPoseEstimator(models_directory, faces_directory, 
                                use_neural_compute_stick=use_neural_compute_stick)

    identifier = fr.FaceReidentificator(models_directory, faces_directory, 
                        use_neural_compute_stick=use_neural_compute_stick)

    filelist = glob.glob(os.path.join(faces_directory, '*.jpg'))
    for filename in filelist:
        print('processing ' + filename)
        rgb_image = cv2.imread(filename)
        ori_image = rgb_image.copy()
                
        boxes = detector.detect_faces(rgb_image)
        print('number of boxes ' + str(len(boxes)))
        for bounding_box in boxes: # in each face
            # normalized landmarks (0-1) and cropped faces
            landmarks, landmark_names, face_crop = detector.detect_normalized_facial_landmarks_and_face_crop(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            if debug_output == True:
                detector.draw_bounding_box(ori_image, bounding_box)
                cv2.imshow("Image Window", ori_image)
                cv2.waitKey(1000)
            
            # face_shape is the shape of face area which we use to calculate face landmark.
            face_shape = face_crop.shape
            face_crop_aligned = identifier.align_faces(face_crop, face_shape, landmarks)
        
        features = identifier.calculate_embeddings(face_crop_aligned)
        out_path = args.output
        filename = filename[0:-4]
        outfile = os.path.join(out_path, filename+'.npy')
        print(outfile)
        np.save(outfile, features)
