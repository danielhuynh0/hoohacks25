#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from keras.models import load_model

def calculate_position_penalty(position_info, center_penalty=0.25, side_penalty=0.1):

    penalty = 0.0

    if position_info['has_center']:
        center_weight = min(1.0, position_info['center'] / 30)
        penalty = max(penalty, center_penalty * center_weight)

    if position_info['has_left']:
        left_weight = min(1.0, position_info['left'] / 30)
        penalty = max(penalty, side_penalty * left_weight)

    if position_info['has_right']:
        right_weight = min(1.0, position_info['right'] / 30)
        penalty = max(penalty, side_penalty * right_weight)

    return penalty

def draw_confidence_bar(frame, confidence_score, position_info=None, has_top_activity=False,
                    center_penalty_factor=0.25, side_penalty_factor=0.1, bonus_factor=0.1, threshold=0.8):
    height, width = frame.shape[:2]


    original_confidence = confidence_score

    if has_top_activity and position_info:

        position_based_penalty = calculate_position_penalty(
            position_info, center_penalty_factor, side_penalty_factor)


        confidence_score = max(0, confidence_score - (confidence_score * position_based_penalty))
    elif not has_top_activity:

        bonus = confidence_score * bonus_factor
        confidence_score = min(1.0, confidence_score + bonus)

    overlay = frame.copy()
    overlay_height = int(height * 0.12)
    overlay_y = height - overlay_height
    cv2.rectangle(overlay,
                (0, overlay_y),
                (width, height),
                (20, 20, 20),
                -1)

    alpha = 0.85
    cv2.addWeighted(overlay[overlay_y:height, 0:width], alpha,
                frame[overlay_y:height, 0:width], 1-alpha,
                0, frame[overlay_y:height, 0:width])

    bar_height = int(height * 0.025)
    max_bar_width = int(width * 0.6)
    bar_width = int(max_bar_width * confidence_score)
    bar_start_x = int(width * 0.25)
    bar_start_y = height - int(height * 0.06)

    bg_color = (60, 60, 60)
    cv2.rectangle(frame,
                (bar_start_x, bar_start_y),
                (bar_start_x + max_bar_width, bar_start_y + bar_height),
                bg_color,
                -1)

    if confidence_score >= threshold:

        r = int(255 * (1 - (confidence_score - threshold) / (1 - threshold)))
        g = 255
        b = 0
        color = (b, g, r)
    else:
        r = 255
        g = int(255 * (confidence_score / threshold))
        b = 0
        color = (b, g, r)

    if bar_width > 0:
        cv2.rectangle(frame,
                    (bar_start_x, bar_start_y),
                    (bar_start_x + bar_width, bar_start_y + bar_height),
                    color,
                    -1)

    highlight_thickness = max(1, int(bar_height * 0.2))
    cv2.line(frame,
            (bar_start_x, bar_start_y),
            (bar_start_x + max_bar_width, bar_start_y),
            (120, 120, 120), highlight_thickness)

    percentage = f"{int(confidence_score * 100)}%"
    label = "Confidence"

    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale_label = 0.65
    font_scale_percentage = 0.85
    font_color = (240, 240, 240)
    font_thickness = 1

    text_percentage = percentage
    text_size = cv2.getTextSize(text_percentage, font, font_scale_percentage, font_thickness)[0]
    text_x = bar_start_x + max_bar_width + 15
    text_y = bar_start_y + bar_height // 2 + text_size[1] // 2

    text_label = label
    label_size = cv2.getTextSize(text_label, font, font_scale_label, font_thickness)[0]
    label_x = bar_start_x - label_size[0] - 15
    label_y = bar_start_y + bar_height // 2 + label_size[1] // 2

    if label_x < 10:
        shift_amount = 10 - label_x
        bar_start_x += shift_amount
        label_x = 10
        text_x += shift_amount

    cv2.putText(frame, text_percentage, (text_x+1, text_y+1), font,
                font_scale_percentage, (0, 0, 0), font_thickness + 1)
    cv2.putText(frame, text_percentage, (text_x, text_y), font,
                font_scale_percentage, font_color, font_thickness)

    cv2.putText(frame, text_label, (label_x+1, label_y+1), font,
                font_scale_label, (0, 0, 0), font_thickness + 1)
    cv2.putText(frame, text_label, (label_x, label_y), font,
                font_scale_label, font_color, font_thickness)

    for i in range(1, 10):
        tick_x = bar_start_x + (max_bar_width * i) // 10
        tick_height = bar_height // 3
        tick_color = (80, 80, 80)
        cv2.line(frame,
                (tick_x, bar_start_y + bar_height - tick_height),
                (tick_x, bar_start_y + bar_height),
                tick_color, 1)

    return frame


class WeedDetector(Node):
    def __init__(self):
        super().__init__('weed_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        model_path = "/home/hack/ros2_ws/src/weed_detector/weed_detector/deepweeds_binary_final.h5"
        self.model = load_model(model_path)
        self.get_logger().info(f"Weed detection model loaded from {model_path}")
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Image recieved!')
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {str(e)}")
            return

        img_resized = cv2.resize(cv_img, (224, 224))
        img_array = img_resized.astype(np.float32) / 255.0
        img_array = np.expand_dims(img_array, axis=0)

        prediction = self.model.predict(img_array)[0][0]
        label = "Weed" if prediction > 0.5 else "Not Weed"
        confidence = prediction if prediction > 0.5 else 1 - prediction
        result_str = f"{label} ({confidence * 100:.2f}%)"
        self.get_logger().info(f"Prediction: {result_str}")
        frame = draw_confidence_bar(cv_img, confidence)
        # self.result_pub.publish(String(data=result_str))
        # self.get_logger().info(f"Prediction: {result_str}")

        # gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        # _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        # contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # if contours:
        #     largest_contour = max(contours, key=cv2.contourArea)
        #     x, y, w, h = cv2.boundingRect(largest_contour)  

        #     cv2.rectangle(cv_img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        #     cv2.putText(cv_img, "Main Object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("Main Object Localization", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = WeedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()