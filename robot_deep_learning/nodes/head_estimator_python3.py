#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import deep_models_shared_python3 as dm
import os
import glob

class HeadPoseEstimator:
    def __init__(self, models_directory, faces_directory, use_neural_compute_stick=False):
        # Load the models
        models_dir = models_directory
        print('Using the following directory to load object detector models:', models_dir)
        
        # ========================= head/face detection model ============================= # 
        # file with network architecture and other information
        head_detection_model_prototxt_filename = models_dir + '/head_detection/deploy.prototxt'
        # file with network weights
        head_detection_model_caffemodel_filename = models_dir + '/head_detection/res10_300x300_ssd_iter_140000.caffemodel'
        self.face_confidence_threshold = 0.3

        print('attempting to load neural network from files')
        print('prototxt file =', head_detection_model_prototxt_filename)
        print('caffemodel file =', head_detection_model_caffemodel_filename)
        self.head_detection_model = cv2.dnn.readNetFromCaffe(head_detection_model_prototxt_filename, head_detection_model_caffemodel_filename)
        dm.print_model_info(self.head_detection_model, 'head_detection_model')
        
        # attempt to use Neural Compute Stick 2
        if use_neural_compute_stick:
            print('HeadPoseEstimator.__init__: Attempting to use an Intel Neural Compute Stick 2 using the following command: self.head_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)')
            self.head_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

        # ========================= head pose model ============================= # 
        head_pose_model_dir = models_dir + '/open_model_zoo/head-pose-estimation-adas-0001/FP32/'
        head_pose_weights_filename = head_pose_model_dir + 'head-pose-estimation-adas-0001.bin'
        head_pose_config_filename = head_pose_model_dir + 'head-pose-estimation-adas-0001.xml'
        self.head_pose_model = cv2.dnn.readNet(head_pose_weights_filename, head_pose_config_filename)
        
        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for head pose estimation due to potential errors.')

        dm.print_model_info(self.head_pose_model, 'head_pose_model')

        # ========================= facial landmarks detection model ============================= # 
        landmarks_model_dir = models_dir + '/open_model_zoo/facial-landmarks-35-adas-0002/FP32/'
        landmarks_weights_filename = landmarks_model_dir + 'facial-landmarks-35-adas-0002.bin'
        landmarks_config_filename = landmarks_model_dir + 'facial-landmarks-35-adas-0002.xml'
        self.landmarks_model = cv2.dnn.readNet(landmarks_weights_filename, landmarks_config_filename)

        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for facial landmarks due to potential errors.')

        dm.print_model_info(self.head_pose_model, 'head_pose_model')

        
        dm.print_model_info(self.landmarks_model, 'landmarks_model')
        
        self.landmark_names = ['right_eye_left', 'right_eye_right',
                               'left_eye_right', 'left_eye_left', 'nose_tip',
                               'nose_bottom', 'nose_right', 'nose_left', 'mouth_right',
                               'mouth_left', 'mouth_top', 'mouth_bottom',
                               'right_eyebrow_right', 'right_eyebrow_middle', 'right_eyebrow_left',
                               'left_eyebrow_right', 'left_eyebrow_middle', 'left_eyebrow_left',
                               'right_cheek_18', 'right_cheek_19', 'right_cheek_20', 'right_cheek_21',
                               'right_cheek_22', 'right_cheek_23', 'right_cheek_24',
                               'chin_right', 'chin_middle', 'chin_left',
                               'left_cheek_28', 'left_cheek_29', 'left_cheek_30', 'left_cheek_31',
                               'left_cheek_32', 'left_cheek_33', 'left_cheek_34']


        # Load the models
        models_dir = models_directory
        self.faces_dir = faces_directory
        # ========================= face identification ============================= # 
        faceID_model_dir = models_directory + '/open_model_zoo/face-reidentification-retail-0095/FP32/'
        faceID_weights_filename = faceID_model_dir + 'face-reidentification-retail-0095.bin'
        faceID_config_filename = faceID_model_dir + 'face-reidentification-retail-0095.xml'
        
        self.faceID_model = cv2.dnn.readNet(faceID_weights_filename, faceID_config_filename)
        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for facial landmarks due to potential errors.')

        dm.print_model_info(self.faceID_model, 'faceId_net_model')
        self.cosine_similarity_ths = 0.4 # Compare similarity threshold
        # load precalculated face embeddings
        self.ground_truth_features = self.loadEmbeddings()
    
    def get_landmark_names(self):
        return self.landmark_names

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return None

    def detect_faces(self, rgb_image):
        orig_h, orig_w, c = rgb_image.shape
        face_image = rgb_image
        rot_h, rot_w, c = face_image.shape
        # Assumes that the width is smaller than the height, and crop
        # a width x width square image from the top.
        square_face_image = face_image[:rot_w, :, :]
        sqr_h, sqr_w, c = square_face_image.shape
        network_image = cv2.resize(square_face_image, (300, 300))
        # Some magic numbers came from
        # https://www.pyimagesearch.com/2018/02/26/face-detection-with-opencv-and-deep-learning/
        face_image_blob = cv2.dnn.blobFromImage(network_image, 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.head_detection_model.setInput(face_image_blob)
        face_detections = self.head_detection_model.forward()[0,0,:,:]
        confidence_mask = face_detections[:, 2] > self.face_confidence_threshold
        face_detections = face_detections[confidence_mask]
        coordinates = face_detections[:, 3:7]
        # Scale and rotate coordinates to the original image
        coordinates = coordinates * np.array([sqr_w, sqr_h, sqr_w, sqr_h])
        face_id = 0
        boxes = []
        for x0, y0, x1, y1 in coordinates:
            res = self.check_false_detections(sqr_h, sqr_w, x0, y0, x1, y1)
            if not res:
                print("Got false detection")
                continue
            orig_y0 = y0
            orig_y1 = y1
            orig_x0 = x0
            orig_x1 = x1
            face_id += 1
            bounding_box = [orig_x0, orig_y0, orig_x1, orig_y1]
            boxes.append(bounding_box)

        return boxes
    
    def check_false_detections(self, ih, iw, fsx, fsy, fex, fey):
        if ih > iw:
            if fsx > ih:
                return False
            elif fsy > ih:
                return False
            elif fex > ih:
                return False
            elif fey > ih:
                return False
            else:
                return True
        else:
            if fsx > iw:
                return False
            elif fsy > iw:
                return False
            elif fex > iw:
                return False
            elif fey > iw:
                return False
            else:
                return True

    
    def get_sub_image(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        if enlarge_box:
            scale = enlarge_scale
            orig_h, orig_w, c = rgb_image.shape

            x0 = bounding_box[0]
            y0 = bounding_box[1]
            x1 = bounding_box[2]
            y1 = bounding_box[3]

            m_x = (x1 + x0) / 2.0
            m_y = (y1 + y0) / 2.0

            b_w = x1 - x0
            b_h = y1 - y0

            b_w = scale * b_w
            b_h = scale * b_h

            x0 = int(round(m_x - (b_w/2.0)))
            x1 = int(round(m_x + (b_w/2.0)))
            y0 = int(round(m_y - (b_h/2.0)))
            y1 = int(round(m_y + (b_h/2.0)))

            x0 = max(0, x0)
            x1 = min(orig_w, x1)
            y0 = max(0, y0)
            y1 = min(orig_h, y1)
        else: 
            x0 = int(round(bounding_box[0]))
            y0 = int(round(bounding_box[1]))
            x1 = int(round(bounding_box[2]))
            y1 = int(round(bounding_box[3]))

        actual_bounding_box = [x0, y0, x1, y1]
        image_to_crop = rgb_image
        sub_image = image_to_crop[y0:y1, x0:x1, :]
        return sub_image, actual_bounding_box
    

    def estimate_head_pose(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        face_crop_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=enlarge_box, enlarge_scale=enlarge_scale)
        sqr_h, sqr_w, c = face_crop_image.shape

        if (sqr_h > 0) and (sqr_w > 0):
            head_pose_image_blob = cv2.dnn.blobFromImage(face_crop_image,
                                                         size=(60, 60),
                                                         swapRB=False,
                                                         crop=False,
                                                         ddepth=cv2.CV_32F)
            self.head_pose_model.setInput(head_pose_image_blob)
            head_pose_out = self.head_pose_model.forward(['angle_r_fc', 'angle_p_fc', 'angle_y_fc'])
            rpy = head_pose_out
            roll = rpy[0][0][0]
            pitch = rpy[1][0][0]
            yaw = rpy[2][0][0]
            pitch = pitch * np.pi/180.0
            roll = roll * np.pi/180.0
            yaw = yaw * np.pi/180.0
            
            return yaw, pitch, roll

        return None, None, None


    def detect_normalized_facial_landmarks_and_face_crop(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        face_crop_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=enlarge_box, enlarge_scale=enlarge_scale)
        sqr_h, sqr_w, c = face_crop_image.shape

        if (sqr_h > 0) and (sqr_w > 0):
            landmarks_image_blob = cv2.dnn.blobFromImage(face_crop_image,
                                                         size=(60, 60),
                                                         swapRB=False,
                                                         crop=False,
                                                         ddepth=cv2.CV_32F)
            self.landmarks_model.setInput(landmarks_image_blob)
            landmarks_out = self.landmarks_model.forward()

            s = landmarks_out.shape
            out = np.reshape(landmarks_out[0], (s[1]//2, 2))
            landmarks = {}
            for n, v in enumerate(out):
                x = v[0]
                y = v[1]
                name = self.landmark_names[n]
                landmarks[name] = (x,y)
            return landmarks, self.landmark_names.copy(), face_crop_image
        return None, None


    def detect_facial_landmarks(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        face_crop_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=enlarge_box, enlarge_scale=enlarge_scale)
        sqr_h, sqr_w, c = face_crop_image.shape

        if (sqr_h > 0) and (sqr_w > 0):
            landmarks_image_blob = cv2.dnn.blobFromImage(face_crop_image,
                                                         size=(60, 60),
                                                         swapRB=False,
                                                         crop=False,
                                                         ddepth=cv2.CV_32F)
            self.landmarks_model.setInput(landmarks_image_blob)
            landmarks_out = self.landmarks_model.forward()
            s = landmarks_out.shape
            out = np.reshape(landmarks_out[0], (s[1]//2, 2))
            x0, y0, x1, y1 = actual_bounding_box
            landmarks = {}
            landmarks_normalized = {}
            for n, v in enumerate(out):
                x = int(round((v[0] * sqr_w) + x0))
                y = int(round((v[1] * sqr_h) + y0))
                name = self.landmark_names[n]
                landmarks[name] = (x,y)
                x_n = v[0]
                y_n = v[1]
                landmarks_normalized[name] = (x_n,y_n)

            return landmarks, landmarks_normalized, self.landmark_names.copy()
        return None, None

    def draw_bounding_box(self, image, bounding_box):
        x0 = int(round(bounding_box[0]))
        y0 = int(round(bounding_box[1]))
        x1 = int(round(bounding_box[2]))
        y1 = int(round(bounding_box[3]))
        color = (0, 0, 255)
        thickness = 2
        cv2.rectangle(image, (x0, y0), (x1, y1), color, thickness)

        
    def draw_head_pose(self, image, yaw, pitch, roll, bounding_box):
        x0, y0, x1, y1 = bounding_box
        face_x = (x1 + x0) / 2.0
        face_y = (y1 + y0) / 2.0
        #
        # opencv uses right-handed coordinate system
        # x points to the right of the image
        # y points to the bottom of the image
        # z points into the image
        #

        h, w, c = image.shape
        camera_center = (w/2.0, h/2.0)
        #For rendering with an unknown camera
        focal_length = 50.0 
        camera_matrix = np.array([[focal_length, 0.0,          camera_center[0]],
                                  [0.0,          focal_length, camera_center[1]],
                                  [0.0,          0.0,          1.0]])
        face_translation = np.array([0.0, 0.0, 3000.0])
        distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0])
        # negate the directions of the y and z axes
        axes = np.array([[2000.0,  0.0,      0.0   ],
                         [0.0,     -2000.0,  0.0   ],
                         [0.0,     0.0,      -2000.0],
                         [0.0,     0.0,      0.0   ]])
        head_ypr = np.array([-yaw, pitch, roll])
        rotation_mat = Rotation.from_euler('yxz', head_ypr).as_dcm()
        rotation_vec, jacobian = cv2.Rodrigues(rotation_mat)
        image_points, jacobian = cv2.projectPoints(axes, rotation_vec, face_translation, camera_matrix, distortion_coefficients)
        face_pix = np.array([face_x, face_y])

        origin = image_points[3].ravel()
        x_axis = (image_points[0].ravel() - origin) + face_pix
        y_axis = (image_points[1].ravel() - origin) + face_pix
        z_axis = (image_points[2].ravel() - origin) + face_pix

        p0 = tuple(np.int32(np.round(face_pix)))
        p1 = tuple(np.int32(np.round(x_axis)))
        cv2.line(image, p0, p1, (0, 0, 255), 2)
        p1 = tuple(np.int32(np.round(y_axis)))
        cv2.line(image, p0, p1, (0, 255, 0), 2)
        p1 = tuple(np.int32(np.round(z_axis)))
        cv2.line(image, p0, p1, (255, 0, 0), 2)

        
    def draw_landmarks(self, image, landmarks):
        for name, xy in landmarks.items():
            x = xy[0]
            y = xy[1]
            if 'mouth' in name:
                color = (255, 0, 0)
            elif 'nose' in name:
                color = (0, 255, 0)
            elif 'eyebrow' in name:
                color = (0, 0, 0)
            elif 'right_eye' in name:
                color = (255, 255, 0)
            elif 'left_eye' in name:
                color = (0, 255, 255)
            elif 'chin' in name:
                color = (255, 0, 255)
            else:
                color = (0, 0, 255)
            cv2.circle(image, (x,y), 2, color, 1)
            font_scale = 1.0
            line_color = [0, 0, 0]
            line_width = 1
            font = cv2.FONT_HERSHEY_PLAIN 


    def get_cropped_faces(self, rgb_image, bounding_boxes):
        faces = []
        for bounding_box in bounding_boxes:
            sub_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            faces.append(sub_image)
        return faces

    def loadEmbeddings(self):
        filelist = glob.glob(os.path.join(self.faces_dir, '*.npy'))
        features = {}
        for filename in filelist:
            embedding = np.load(filename)
            # Split the path in head and tail pair 
            head_tail = os.path.split(filename)
            # only tail part of the path
            person_id = head_tail[1][0:-4]
            features[person_id] = embedding
        return features

    ''' 
        INputs
        Name: "data" , shape: [1x3x128x128] - An input image in the format [BxCxHxW], where:
            B - batch size
            C - number of channels
            H - image height
            W - image width

        Expected color order is BGR.

        Ouputs:
        The net outputs a blob with the shape [1, 256, 1, 1], containing a row-vector of 256 floating point values. Outputs on different images are comparable in cosine distance.
    '''
    def calculate_embeddings(self, face_crop):
        features_blob = cv2.dnn.blobFromImage(face_crop,
                                        size=(128, 128),
                                        swapRB=False,
                                        crop=False,
                                        ddepth=cv2.CV_32F)
        self.faceID_model.setInput(features_blob)
        features = self.faceID_model.forward()
        features = features[0,:,0,0]
        return features
    
    def align_faces(self, face_crop, face_shape, landmarks, debug_output=True):
        # left eye, right eye, tip of nose, left lip corner, right lip corner
        landmark_reference = np.float32([(0.68262291666666670, 0.4615741071428571),
                                        (0.31556875000000000, 0.4615741071428571),
                                        (0.50026249999999990, 0.6405053571428571),
                                        (0.65343645833333330, 0.8246919642857142),
                                        (0.34947187500000004, 0.8246919642857142)])                     
        landmark_ref = landmark_reference * np.float32([face_shape[1], face_shape[0]])
        # here is the output of landmark network
        landmark_filtered = np.float32([landmarks['left_eye_left'], landmarks['right_eye_right'], 
                landmarks['nose_tip'], landmarks['mouth_left'], landmarks['mouth_right']]) * np.float32([face_shape[1],face_shape[0]])
        if debug_output:
            print(landmark_filtered)
            print(landmark_ref)
        # Get transformation Matrix
        M = cv2.getAffineTransform(landmark_filtered[0:3], landmark_ref[0:3])
        face_crop_aligned = cv2.warpAffine(face_crop, M, (face_shape[1], face_shape[0]))
        if debug_output:
            cv2.imshow("transform", face_crop_aligned)
        return face_crop_aligned
       

    def draw_names(self, rgb_image):
        cv2.putText(rgb_image, 'truth', (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)

    def apply_to_image(self, rgb_image, draw_output=False):
        if draw_output:
            output_image = rgb_image.copy()
        else:
            output_image = None

        heads = []
        boxes = self.detect_faces(rgb_image)

        for bounding_box in boxes:
            if draw_output: 
                self.draw_bounding_box(output_image, bounding_box)
            yaw, pitch, roll = self.estimate_head_pose(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            if yaw is not None: 
                ypr = (yaw, pitch, roll)
                if draw_output: 
                    self.draw_head_pose(output_image, yaw, pitch, roll, bounding_box)
            else:
                ypr = None
            landmarks, landmarks_normalized, landmark_names = self.detect_facial_landmarks(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            if (landmarks is not None) and draw_output: 
                self.draw_landmarks(output_image, landmarks)

            cropped_face, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15) 
            cropped_face = cv2.resize(cropped_face, (128,128))
            face_crop_aligned = self.align_faces(cropped_face, cropped_face.shape, landmarks_normalized, debug_output=False)

            feature = self.calculate_embeddings(face_crop_aligned)
            test_length = np.linalg.norm(feature)
            for id, gt in self.ground_truth_features.items():
                ground_truth_length = np.linalg.norm(gt) # The length of feature
                similarity = np.dot(gt, feature)/(ground_truth_length * test_length)   
                if similarity > self.cosine_similarity_ths:
                    if draw_output:
                        pos = (int((bounding_box[0]+bounding_box[2])/2), int(bounding_box[3]+50))
                        #print(pos)
                        cv2.putText(output_image, id, pos , cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1, cv2.LINE_AA)
                    break # face matched, break
            heads.append({'box':bounding_box, 'ypr':ypr, 'landmarks':landmarks, 'faceID': id})
        return heads, output_image
