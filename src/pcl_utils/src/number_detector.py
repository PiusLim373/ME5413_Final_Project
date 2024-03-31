#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from pcl_utils.srv import (
    GetPose,
    GetPoseResponse,
    GetPoseFromPixelRequest,
    GetPoseFromPixel,
)
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pytesseract
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class NumberDetector:
    def __init__(self):
        # Subscribers
        rospy.Subscriber(
            "/camera_front/color/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
        )

        # Services
        rospy.Service("get_pose", GetPose, self.get_pose_cb)
        self.get_map_pose_srv = rospy.ServiceProxy("get_camera_pose_from_pixel", GetPoseFromPixel)

        # Publishers
        self.number_detector_output_pub = rospy.Publisher("number_detector_output", Image, queue_size=1)
        self.goal_pose_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # Class variables
        self.bridge = CvBridge()
        self.cv_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_pose_cb(self, req):
        rospy.loginfo(f"[Number Detector] service called to detect number {req.box_number}")
        res = GetPoseResponse()

        # Convert the image to grayscale
        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        # Apply thresholding to get binary image and contours
        _, binary_image = cv2.threshold(gray_image, 30, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Iterate the contours and filter out those might not be numbers
        digit_contours = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)
            area = cv2.contourArea(contour)
            if aspect_ratio > 0.2 and aspect_ratio < 1.5 and area > 100:
                digit_contours.append(contour)
        all_digit_detected_data = {}
        image_to_pub = self.cv_image
        # Loop through all contours and process abit before passing to pytesseract
        for contour in digit_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cropped_digit_img = binary_image[y : y + h, x : x + w]
            cropped_digit_img_inverted = 255 - cropped_digit_img
            padded_digit_img = cv2.copyMakeBorder(
                cropped_digit_img_inverted,
                5,
                5,
                5,
                5,
                cv2.BORDER_CONSTANT,
                value=255,
            )
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            cropped_digit_img_eroded = cv2.erode(padded_digit_img, kernel)

            digit_text = pytesseract.image_to_string(
                cropped_digit_img_eroded,
                config="--psm 10 oem 3 -c tessedit_char_whitelist=0123456789",
            )
            digit_text = digit_text.strip()
            # draw green box for non target number detection, red for targetted
            if digit_text:
                color = (0, 255, 0)
                if digit_text == str(req.box_number):
                    color = (0, 0, 255)
                cv2.rectangle(image_to_pub, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    image_to_pub,
                    digit_text,
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2,
                    color,
                    2,
                )
                # if we looks at the box at a certain angle, might get 2 detection, will save both and average later
                u = int(x + 0.5 * w)
                v = int(y + 0.5 * h)
                if digit_text in all_digit_detected_data.keys():
                    all_digit_detected_data[digit_text].append((u, v))
                else:
                    all_digit_detected_data[digit_text] = [(u, v)]
        rospy.loginfo(f"[Number Detector] all digit data: {all_digit_detected_data}")
        # publish out anotated image for visualization
        ros_image_msg = self.bridge.cv2_to_imgmsg(image_to_pub, encoding="bgr8")
        self.number_detector_output_pub.publish(ros_image_msg)

        # if dont have number of interest, return with successcode 0
        if not str(req.box_number) in all_digit_detected_data.keys():
            res.success_code = GetPoseResponse().FAILED_WITH_NO_DETECTION
            return res
        # average the pixel count for multiple detection, changes are we are looking the box at a slanted angle
        i = 0
        sum_u = 0
        sum_v = 0
        for x in all_digit_detected_data[str(req.box_number)]:
            sum_u += x[0]
            sum_v += x[1]
            i += 1
        average_u = int(sum_u / i)
        average_v = int(sum_v / i)
        rospy.loginfo(
            f"[Number Detector] Found {i} indicies of number {req.box_number} box, after averaging, (u,v): {average_u, average_v}"
        )

        # call the pointcloud transformer to get coor_wrt_camera from pixel count
        get_map_pose_req = GetPoseFromPixelRequest()
        get_map_pose_req.u = average_u
        get_map_pose_req.v = average_v
        get_map_pose_res = self.get_map_pose_srv.call(get_map_pose_req)

        # transform coor_wrt_camera to coor_wrt_map
        if get_map_pose_res.success:

            transform = self.tf_buffer.lookup_transform(
                "map",
                "camera_front_depth_optical_frame",
                rospy.Time(0),
                rospy.Duration(2.0),
            )
            coor_wrt_map = do_transform_pose(get_map_pose_res.coor_wrt_camera, transform)
            data = PoseStamped()
            data = coor_wrt_map
            data.header.frame_id = "map"
            data.pose.position.z = 0
            data.pose.orientation.x = 0.0
            data.pose.orientation.y = 0.0
            data.pose.orientation.z = 0.0
            data.pose.orientation.w = 1.0
            res.success_code = GetPoseResponse().SUCCESS
            res.coor_wrt_map = coor_wrt_map

            # ===================================================================== this part only for debugging, shouldnt directly publish goal to move base here!!!
            self.goal_pose_pub.publish(data)
            rospy.loginfo(f"[Number Detector] goal send to move base: {data}")

            return res

        # sometime if box is too far, although detected, pointcloud cant find the distance, will return either at left/mid/right side of the robot
        else:
            if average_u < 440:
                res.success_code = GetPoseResponse().FAILED_WITH_NUMBER_DETECTED_LEFT
            elif average_u >= 440 and average_u <= 840:
                res.success_code = GetPoseResponse().FAILED_WITH_NUMBER_DETECTED_MID
            else:
                res.success_code = GetPoseResponse().FAILED_WITH_NUMBER_DETECTED_RIGHT
            return res

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[Number Detector] {e}")


if __name__ == "__main__":
    rospy.init_node("number_detector")
    number_detector = NumberDetector()
    rospy.spin()
