import cv2
import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import time
import keyboard

from open3d import create_rgbd_image_from_color_and_depth
from open3d import read_point_cloud

#Default config parametri L515
"""
    rs.stream.depth -> width:640, height:480, rs.format:z16, framerate:30
    rs.stream.color -> width:960, height:540, rs.format:bgr8, framerate:30
"""

def Translation(x, y, z):
    return [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [x, y, z, 1]]

def _Translation(x, y, z):
    return [[x],
            [y],
            [z]]

def Scale(x, y, z):
    return [[x, 0, 0, 0],
            [0, y, 0, 0],
            [0, 0, z, 0],
            [0, 0, 0, 1]]

def X_Rotation(angle):
    return [[1, 0, 0, 0],
           [0, np.cos(angle), np.sin(angle), 0],
           [0, -np.sin(angle), np.cos(angle), 0],
           [0, 0, 0, 1]]

def Y_Rotation(angle):
    return [[np.cos(angle), 0, -np.sin(angle), 0],
            [0, 1, 0, 0],
            [np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]]

def Z_Rotation(angle):
    return [[np.cos(angle), np.sin(angle), 0, 0],
            [-np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1]]

class Scan():
                #(self, width, height, framerate, autoexposureFrames, backDistance):
    def __init__(self):
        #Kreiranje osnovnog PCD-a
        #self.main_pcd = o3d.geometry.PointCloud()

        #Startovanje/Konfigurisanje L515
        self.pipe = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)

        #Post-Proccessing filteri
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)
        self.dec_filter = rs.decimation_filter()
        self.temp_filter = rs.temporal_filter()
        self.spat_filter = rs.spatial_filter()
        self.hole_filter = rs.hole_filling_filter()
        self.threshold = rs.threshold_filter(0.17, 0.4)

    def startPipeline(self):
        self.pipe.start(self.config)
        self.align = rs.align(rs.stream.color)
        print("Pipeline started!", end="\n")

    def stopPipeline(self):
        self.pipe.stop()
        self.pipe = None
        self.config = None
        print("Pipeline stopped", end="\n")

    def takeFoto(self):
        #print("Taking a photo!", end="\n")

        self.frameset = self.pipe.wait_for_frames()
        self.frameset = self.align.process(self.frameset)
        self.profile = self.frameset.get_profile()

      #  self.dpth =

        """
        coeffs	-> Distortion coefficients
        fx	    -> Focal length of the image plane, as a multiple of pixel width
        fy	    -> Focal length of the image plane, as a multiple of pixel height
        height	-> Height of the image in pixels
        model	-> Distortion model of the image
        ppx	    -> Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge
        ppy	    -> Vertical coordinate of the principal point of the image, as a pixel offset from the top edge
        width	-> Width of the image in pixels
        """

        self.depth_intrinsics = self.profile.as_video_stream_profile().get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.fx, self.fy = self.depth_intrinsics.fx,self.depth_intrinsics.fy
        self.px, self.py = self.depth_intrinsics.ppx,self.depth_intrinsics.ppy

        self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()

        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w,self.h,self.fx,self.fy,self.px,self.py)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())

    def processFoto(self):

        self.depth_frame_open3d = o3d.geometry.Image(self.depth_image)
        self.color_frame_open3d = o3d.geometry.Image(self.color_image)

        self.rgbd_image = create_rgbd_image_from_color_and_depth(self.color_frame_open3d,
                                                                 self.depth_frame_open3d,
                                                                 convert_rgb_to_intensity=False)
        return o3d.geometry.create_point_cloud_from_rgbd_image(self.rgbd_image, self.intrinsic)

    def getPointcloud(self):
        return self.main_pcd


#Testiranje
pcd_main = o3d.geometry.PointCloud()
pcd_front = o3d.geometry.PointCloud()
pcd_back = o3d.geometry.PointCloud()

Skeniranje = Scan()
Skeniranje.startPipeline()
i = 0

while True:
    Skeniranje.takeFoto()
    cv2.imshow("Skeniranje", Skeniranje.color_image)
    if cv2.waitKey(1) == ord("t"):
        print("Creating PCD " + str(i) + "!", end="\n")
        o3d.io.write_point_cloud("out" + str(i) + ".pcd", Skeniranje.processFoto())
        i+=1
    elif keyboard.is_pressed("q"):
        cv2.destroyAllWindows()
        break


for pcd in range(0, i):
    cloud = o3d.io.read_point_cloud("./out" + str(pcd) + ".pcd")
    o3d.visualization.draw_geometries([cloud], "PointCloud" + str(pcd), 720, 576)

Skeniranje.stopPipeline()
