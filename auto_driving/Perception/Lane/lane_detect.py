#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import random
import tf

from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped, Point
import warnings

warnings.filterwarnings("ignore")

params_cam ={
        "ENGINE": "UNITY",
        "TYPE": "JPG",
        "WIDTH": 640,
        "HEIGHT": 480,
        "FOV": 60,
        "X": 3.50,
        "Y": 0,
        "Z": 0.50,
        "YAW": 0,
        "PITCH": 0.0,
        "ROLL": 0
}


class IMGParser:
    def __init__(self, pkg_name='ssafy_ad'):
        print("Image Parser INIT!")
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        # self.ego_sub = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/lane_path', Path, queue_size=30)

        self.img_bgr = None
        self.img_lane = None
        self.edges = None
        self.is_status = False

        # sementic
        
        self.lower_wlane = np.array([250, 250, 250])
        self.upper_wlane = np.array([255, 255, 255])
        self.lower_rlane = np.array([0, 0, 250])
        self.upper_rlane = np.array([0, 0, 255])

        # # intensity
        # self.lower_wlane = np.array([0,0,205])
        # self.upper_wlane = np.array([30,60,255])

        # self.lower_ylane = np.array([0,70,120])# ([0,60,100])
        # self.upper_ylane = np.array([40,195,230])# ([40,175,255])

        self.crop_pts = np.array([[[0, 480], [0, 350], [280, 200], [360, 200], [640, 350], [640, 480]]])
        #self.crop_pts = np.array([[[0,350],[280,200],[360,200],[640,350]]])

        bev_op = BEVTransform(params_cam=params_cam)
        curve_learner = CURVEFit(order=3, lane_width=4, dx=0.2, y_margin=0.7, x_range=12, min_pts=3,alpha=1)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_bgr is not None and self.is_status == True:
                img_crop = self.mask_roi(self.img_bgr)
                img_warp = bev_op.warp_bev_img(img_crop)
                img_lane = self.binarize(img_warp)
                img_f = bev_op.warp_inv_img(img_lane)
                lane_pts = bev_op.recon_lane_pts(img_f)
                try:
                    x_pred, y_pred_l, y_pred_r,self.edges = curve_learner.fit_curve(lane_pts)
                    curve_learner.set_vehicle_status(self.status_msg)
                    lane_path = curve_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)
                    self.path_pub.publish(lane_path)
                    left_lane = y_pred_l.mean()
                    right_lane = y_pred_r.mean()
                    if self.edges:
                        print("#########차선 이탈##########")
                        font = cv2.FONT_HERSHEY_SIMPLEX  # 폰트 선택
                        font_scale = 0.5  # 폰트 크기
                        font_color = (0, 0, 255)  # 폰트 색상 (BGR 포맷, 초록색)
                        font_thickness = 2  # 폰트 두께

                        # 이미지에 텍스트 작성
                        text = "Lane Department"  # 작성할 텍스트 내용
                        position = (200, 300)  # 텍스트를 작성할 위치 (x, y 좌표)
                        cv2.putText(img_f, text, position, font, font_scale, font_color, font_thickness)
                    else:
                        if(abs(abs(left_lane) - abs(right_lane)) <0.35):
                            print("=======중앙 유지 중=======")
                        else:
                            if(abs(left_lane) > abs(right_lane)):
                                print("==============오른쪽 치우침")
                            else:
                                print("왼쪽 치우침==============")
                        
                    xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)
                    img_f = self.draw_lane_img(img_lane, xyl[:, 0].astype(np.int32),
                                               xyl[:, 1].astype(np.int32),
                                               xyr[:, 0].astype(np.int32),
                                               xyr[:, 1].astype(np.int32))
                    
                    
                except:
                    continue
                cv2.imshow("Received Image", img_f)
                cv2.waitKey(1)
            else:
                print("Waiting IMG")
            rate.sleep()

    def odom_callback(self, msg):  ## Vehicl Status Subscriber
        self.status_msg = msg
        self.is_status = True

    def ego_callback(self,msg):
        print(msg)

    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

    def binarize(self, img):
# #        intensity Camera
#         img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
#         img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)

        # sementic  Camera
        img_wlane = cv2.inRange(img, self.lower_wlane, self.upper_wlane)
        # self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)

        return img_wlane

    def mask_roi(self, img):

        h, w, c = img.shape
        mask = np.zeros((h, w, c), dtype=np.uint8)

        if len(img.shape) == 3:
            mask_value = (255, 255, 255)
        else:
            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)
        mask = cv2.bitwise_and(mask, img)
        return mask

    def draw_lane_img(self, img, leftx, lefty, rightx, righty):
        '''
        place the lidar points into numpy arrays in order to make intensity map
        \n img : source image
        \n leftx, lefty, rightx, righty : curve fitting result
        '''
        point_np = cv2.cvtColor(np.copy(img), cv2.COLOR_GRAY2BGR)
        # Left Lane
        for ctr in zip(leftx, lefty):
            point_np = cv2.circle(point_np, ctr, 2, (255, 0, 0), -1)

        # Right Lane
        for ctr in zip(rightx, righty):
            point_np = cv2.circle(point_np, ctr, 2, (0, 0, 255), -1)

        for ctr in zip(rightx+leftx, righty+lefty):
            point_np = cv2.circle(point_np, (int(ctr[0]/2),int(ctr[1]/2)), 2, (255, 255, 255), -1)


        return point_np

# Finish
class BEVTransform:
    def __init__(self, params_cam, xb=10.0, zb=10.0):
        self.xb = xb
        self.zb = zb
        self.prev_xyz=None
        # Camera Parmas
        self.theta = np.deg2rad(params_cam["PITCH"])
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.x = params_cam["X"]

        # Half FOV
        self.alpha_r = np.deg2rad(params_cam["FOV"] / 2)

        # focal length
        self.fc_y = params_cam["HEIGHT"] / (2 * np.tan(np.deg2rad(params_cam["FOV"] / 2)))
        self.alpha_c = np.arctan2(params_cam["WIDTH"] / 2, self.fc_y)
        self.fc_x = self.fc_y

        self.h = params_cam["Z"] + 0.34
        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        # BEV to Ground Mat
        # (Move dot x -90 Rot) dot (z 180 RotMat)
        self.RT_b2g = np.matmul(np.matmul(self.traslationMtx(xb, 0, zb), 
                                          self.rotationMtx(np.deg2rad(-90), 0, 0)),
                                          self.rotationMtx(0, 0, np.deg2rad(180)))

        self.proj_mtx = self.project2img_mtx(params_cam)

        self._build_tf(params_cam)

    #finish
    def calc_Xv_Yu(self, U, V):
        # 2D img Pixel(U,V) -> 3D Coord(Xv,Yu)
        Xv = self.h * (np.tan(self.theta) * (1 - 2 * (V - 1) / (self.m - 1)) * np.tan(self.alpha_r) - 1) / \
             (-np.tan(self.theta) + (1 - 2 * (V - 1) / (self.m - 1)) * np.tan(self.alpha_r))

        Yu = (1 - 2 * (U - 1) / (self.n - 1)) * Xv * np.tan(self.alpha_c)

        return Xv, Yu

    # finish
    def _build_tf(self, params_cam): 
        # Get Perspective Mat & Inv Perspective Mat
        v = np.array([params_cam["HEIGHT"] * 0.5, params_cam["HEIGHT"]]).astype(np.float32)
        u = np.array([0, params_cam["WIDTH"]]).astype(np.float32)
        # Make grid with x = U, Y = v
        U, V = np.meshgrid(u, v)

        # Make 2D Pixel -> 3D coord(Xv,Yu)
        Xv, Yu = self.calc_Xv_Yu(U, V)

        # 3D Ground Coord
        xyz_g = np.concatenate([Xv.reshape([1, -1]) + params_cam["X"],
                                Yu.reshape([1, -1]),
                                np.zeros_like(Yu.reshape([1, -1])),
                                np.ones_like(Yu.reshape([1, -1]))], axis=0)

        # (Bird to Ground).dot(Ground) -> Bird
        xyz_bird = np.matmul(np.linalg.inv(self.RT_b2g), xyz_g)
        # Bird -> Image Coord
        xyi = self.project_pts2img(xyz_bird)
        # Origin Coord
        src_pts = np.concatenate([U.reshape([-1, 1]), V.reshape([-1, 1])], axis=1).astype(np.float32)
        # Translate Coord
        dst_pts = xyi.astype(np.float32)

        self.perspective_tf = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.perspective_inv_tf = cv2.getPerspectiveTransform(dst_pts, src_pts)

    # finish
    def warp_bev_img(self, img):
        # IMAGE -> BEV
        img_warp = cv2.warpPerspective(img, self.perspective_tf, (self.width, self.height), flags=cv2.INTER_LINEAR)
        return img_warp

    # finish
    def warp_inv_img(self, img_warp):
        # BEV -> Origin IMG
        img_f = cv2.warpPerspective(img_warp, self.perspective_inv_tf, (self.width, self.height),
                                    flags=cv2.INTER_LINEAR)

        return img_f

    # finish
    def recon_lane_pts(self, img):
        if cv2.countNonZero(img) != 0:
            # Find White pixel = findNonZero(imge)
            UV_mark = cv2.findNonZero(img).reshape([-1, 2]) # (N,2) shape
            # [:,0] select first Value from ALL ROWS
            U, V = UV_mark[:, 0].reshape([-1, 1]), UV_mark[:, 1].reshape([-1, 1])
            # [[num],
            #  [num],
            #     ..]
            # Translate to SomeThing... U & V
            Xv, Yu = self.calc_Xv_Yu(U, V)

            # XYZ ground coordinate
            xyz_g = np.concatenate([Xv.reshape([1, -1]) + self.x,
                                    Yu.reshape([1, -1]),
                                    np.zeros_like(Yu.reshape([1, -1])),
                                    np.ones_like(Yu.reshape([1, -1]))], axis=0)
            # xyz_g[0, :]>=0 -> num>=0 from All data in First Row
            # xyg_g[ : , condition] -> Select columns where the value match with the condition and First Row
            xyz_g = xyz_g[:, xyz_g[0, :] >= 0]

        else:
            xyz_g = np.zeros((4, 10))


        return xyz_g

    # finish
    def project_lane2img(self, x_pred, y_pred_l, y_pred_r):
        # xyz Ground Coord -> Bird Coord -> 2D Image -> Crop
        xyz_l_g = np.concatenate([x_pred.reshape([1, -1]),
                                  y_pred_l.reshape([1, -1]),
                                  np.zeros_like(y_pred_l.reshape([1, -1])),
                                  np.ones_like(y_pred_l.reshape([1, -1]))
                                  ], axis=0)

        xyz_r_g = np.concatenate([x_pred.reshape([1, -1]),
                                  y_pred_r.reshape([1, -1]),
                                  np.zeros_like(y_pred_r.reshape([1, -1])),
                                  np.ones_like(y_pred_r.reshape([1, -1]))
                                  ], axis=0)

        # xyz Bird Coord Translate
        xyz_l_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_l_g)
        xyz_r_b = np.matmul(np.linalg.inv(self.RT_b2g), xyz_r_g)

        # Bird View Coord -> Image Coord
        xyl = self.project_pts2img(xyz_l_b)
        xyr = self.project_pts2img(xyz_r_b)

        # Crop data
        xyl = self.crop_pts(xyl)
        xyr = self.crop_pts(xyr)

        return xyl, xyr

    # print
    def project_pts2img(self, xyz_bird):
        # Bird View Coord -> Image Coord

        # x, y, z reshape(1,-1)
        xc, yc, zc = xyz_bird[0, :].reshape([1, -1]), xyz_bird[1, :].reshape([1, -1]), xyz_bird[2, :].reshape([1, -1])

        # Normalization for Projection Transformation
        # detect from ZERO division, +0.0001
        xn, yn = xc / (zc + 0.0001), yc / (zc + 0.0001)
        # mat mul Project_Matrix @ [[xn],[yn],[1]]
        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))
        # Slice 0~1 rows and Transpose = (X,Y) coord
        xyi = xyi[0:2, :].T

        return xyi

    # finish
    def crop_pts(self, xyi):
        # 0 <= X coord < self.width
        # 0 <= Y coord < self.height
        xyi = xyi[np.logical_and(xyi[:, 0] >= 0, xyi[:, 0] < self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1] >= 0, xyi[:, 1] < self.height), :]

        return xyi

    # finish
    def traslationMtx(self, x, y, z):
        # Move Mat
        M = np.array([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1],
                      ])

        return M

    # finish
    def project2img_mtx(self, params_cam):
        '''
        project the lidar points to 2d plane
        xc, yc, zc : xyz components of lidar points w.r.t a camera coordinate
        params_cam : parameters from cameras

        '''
        # focal lengths
        fc_x = params_cam["HEIGHT"] / (2 * np.tan(np.deg2rad(params_cam["FOV"] / 2)))
        fc_y = params_cam["HEIGHT"] / (2 * np.tan(np.deg2rad(params_cam["FOV"] / 2)))

        # the center of image
        cx = params_cam["WIDTH"] / 2
        cy = params_cam["HEIGHT"] / 2

        # transformation matrix from 3D to 2D
        R_f = np.array([[fc_x, 0, cx],
                        [0, fc_y, cy]])

        return R_f

    # finish
    def rotationMtx(self, yaw, pitch, roll):
        # x axis rotation
        R_x = np.array([[1, 0, 0, 0],
                        [0, math.cos(roll), -math.sin(roll), 0],
                        [0, math.sin(roll), math.cos(roll), 0],
                        [0, 0, 0, 1],
                        ])
        # y axis rotation
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch), 0],
                        [0, 1, 0, 0],
                        [-math.sin(pitch), 0, math.cos(pitch), 0],
                        [0, 0, 0, 1],
                        ])
        # z axis rotation
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0, 0],
                        [math.sin(yaw), math.cos(yaw), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1],
                        ])

        R = np.matmul(R_x, np.matmul(R_y, R_z))

        return R


class CURVEFit:
    def __init__(self, order, lane_width, y_margin, x_range, dx, min_pts,alpha):

        self.order = order
        self.lane_width = lane_width
        self.y_margin = y_margin
        self.x_range = x_range
        self.dx = dx
        self.min_pts = min_pts
        self.alpha=alpha

        self.lane_path = Path()

        self.ransac_left = linear_model.RANSACRegressor(estimator=linear_model.Lasso(alpha=self.alpha,max_iter=1000),
                                                        max_trials=10000000,
                                                        loss='absolute_error',
                                                        min_samples=self.min_pts,
                                                        residual_threshold=self.y_margin)

        self.ransac_right = linear_model.RANSACRegressor(estimator=linear_model.Lasso(alpha=self.alpha,max_iter=1000),
                                                         max_trials=10000000,
                                                         loss='absolute_error',
                                                         min_samples=self.min_pts,
                                                         residual_threshold=self.y_margin)

        self._init_model()

    def _init_model(self):
        # Model Initiate with X data
        # Order = Data length = Lane Minimum Length
        # y_l = left lane X coord
        # y_r = left lane X coord

        # So, Find left lane Model Training with INPUT(Order/X) OUTPUT(y_l)
        # Find right lane Model Training with INPUT(Order/X) OUTPUT(y_r)
        X = np.stack([np.arange(0, 2, 0.02) ** i for i in reversed(range(1, self.order + 1))]).T
        
        y_l = 0.5 * self.lane_width * np.ones_like(np.arange(0, 2, 0.02))
        y_r = -0.5 * self.lane_width * np.ones_like(np.arange(0, 2, 0.02))
        self.ransac_left.fit(X, y_l)
        self.ransac_right.fit(X, y_r)

    def preprocess_pts(self, lane_pts):
        idx_list = []

        # Random Sampling
        # lane_pts : Ground Coord
        for d in np.arange(0, self.x_range, self.dx):
            # D : from 0 to x_range, step:dx
            # In lane_pts' first Row, find the value is bigger than d and smaller than d + dx and change Type to list
            # Finally, get lane_pts' X coord, can get sample data nearby x_range in idx_list
            idx_full_list = np.where(np.logical_and(lane_pts[0, :] >= d, lane_pts[0, :] < d + self.dx))[0].tolist()
            idx_list += random.sample(idx_full_list, np.minimum(self.min_pts, len(idx_full_list)))

        # get idx_lists columns from lane_pts
        lane_pts = lane_pts[:, idx_list]

        x_g = np.copy(lane_pts[0, :])
        y_g = np.copy(lane_pts[1, :])

        # 이전 Frame의 Fitting 정보를 활용하여 현재 Line에 대한 Point를 분류
        X_g = np.stack([x_g ** i for i in reversed(range(1, self.order + 1))]).T

        y_ransac_collect_r = self.ransac_right.predict(X_g)
        # y_right = y_g( np.logical_and(Model results y - y_margin , Model results y + y_margin ) )
        # Select correct data from y_ground
        mask =np.logical_and(y_g >= y_ransac_collect_r - self.y_margin, y_g < y_ransac_collect_r + self.y_margin) 

        y_right = y_g[mask]
        x_right = x_g[mask]

        y_ransac_collect_l = self.ransac_left.predict(X_g)

        mask = np.logical_and(y_g >= y_ransac_collect_l - self.y_margin, y_g < y_ransac_collect_l + self.y_margin)
        y_left = y_g[mask]
        x_left = x_g[mask]

        # 1. Sampling 된 Point들의 X좌표를 활용하여 Prediction Input 생성
        # 2. RANSAC을 활용하여 Prediction 수행
        # 3. 결과로 나온 Y좌표가 실제 값의 Y좌표와 마진보다 작게 차이나면 Left, Right Lane Point 집합에 추가

        return x_left, y_left, x_right, y_right

    def fit_curve(self, lane_pts):
        try:
            x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)
        except:
            return

        if len(y_left)==0 and len(y_right)==0:
            edge=True
        else:
            edge=False

        if len(y_left) == 0 or len(y_right) == 0:
            self._init_model()
            x_left, y_left, x_right, y_right = self.preprocess_pts(lane_pts)

        # Weakness model
        if y_left.shape[0] >= self.ransac_left.min_samples:
            X_left = np.stack([x_left ** i for i in reversed(range(1, self.order + 1))]).T
            self.ransac_left.fit(X_left, y_left)

        # Weakness model
        if y_right.shape[0] >= self.ransac_right.min_samples:
            X_right = np.stack([x_right ** i for i in reversed(range(1, self.order + 1))]).T
            self.ransac_right.fit(X_right, y_right)
            
        x_pred = np.arange(0, self.x_range, self.dx).astype(np.float32)
        X_pred = np.stack([x_pred ** i for i in reversed(range(1, self.order + 1))]).T

        y_pred_l = self.ransac_left.predict(X_pred)
        y_pred_r = self.ransac_right.predict(X_pred)   

        # Good model, update lane width
        if y_left.shape[0] >= self.ransac_left.min_samples and y_right.shape[0] >= self.ransac_right.min_samples:
            self.update_lane_width(y_pred_l, y_pred_r)

        # left model is weak
        if y_left.shape[0] < self.ransac_left.min_samples:
            y_pred_l = y_pred_r + self.lane_width

        # right model is weak
        if y_right.shape[0] < self.ransac_right.min_samples:
            y_pred_r = y_pred_l - self.lane_width

        if len(y_pred_l) == len(y_pred_r):
            if np.mean(y_pred_l + y_pred_r):
                if y_pred_r[x_pred == 3.0] > 0:
                    y_pred_r = y_pred_l - self.lane_width
                elif y_pred_l[x_pred == 3.0] < 0:
                    y_pred_l = y_pred_r + self.lane_width
            else:
                pass
        else:
            pass


        return x_pred, y_pred_l, y_pred_r, edge

    # finish
    def update_lane_width(self, y_pred_l, y_pred_r):
        self.lane_width = np.clip(np.max(y_pred_l - y_pred_r), 3.5, 5)

    # finish
    def write_path_msg(self, x_pred, y_pred_l, y_pred_r, frame_id='/map'):
        self.lane_path = Path()

        trans_matrix = np.array([
            [math.cos(self.vehicle_yaw), -math.sin(self.vehicle_yaw), self.vehicle_pos_x],
            [math.sin(self.vehicle_yaw), math.cos(self.vehicle_yaw), self.vehicle_pos_y],
            [0, 0, 1]])

        self.lane_path.header.frame_id = frame_id
        for i in range(len(x_pred)):
            local_result = np.array([[x_pred[i]], [(0.5) * (y_pred_l[i] + y_pred_r[i])], [1]])
            global_result = trans_matrix.dot(local_result)

            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = global_result[0][0]
            tmp_pose.pose.position.y = global_result[1][0]
            tmp_pose.pose.position.z = 0
            tmp_pose.pose.orientation.x = 0
            tmp_pose.pose.orientation.y = 0
            tmp_pose.pose.orientation.z = 0
            tmp_pose.pose.orientation.w = 1
            self.lane_path.poses.append(tmp_pose)

        return self.lane_path

    # finish
    def set_vehicle_status(self, vehicle_status):

        odom_quaternion = (vehicle_status.pose.pose.orientation.x, vehicle_status.pose.pose.orientation.y,
                           vehicle_status.pose.pose.orientation.z, vehicle_status.pose.pose.orientation.w)

        _, _, vehicle_yaw = tf.transformations.euler_from_quaternion(odom_quaternion)
        self.vehicle_yaw = vehicle_yaw
        self.vehicle_pos_x = vehicle_status.pose.pose.position.x
        self.vehicle_pos_y = vehicle_status.pose.pose.position.y


if __name__ == '__main__':
    print("init_node start")
    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()


