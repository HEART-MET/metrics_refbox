import random
from python_qt_binding.QtWidgets import QCheckBox, QRadioButton, QComboBox
from python_qt_binding import QtGui
from metrics_refbox.benchmark_config import BenchmarkConfig
from metrics_refbox_msgs.msg import ObjectDetectionResult
from rospy_message_converter import message_converter
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ObjectDetectionConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(ObjectDetectionConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)
        self.cv_bridge = CvBridge()

    def generate(self):
        trial_config = super(ObjectDetectionConfig, self).generate()
        # make sure the selected target object is not included in the
        # selected secondary objects
        if trial_config['Target object'][0] in trial_config['Secondary objects']:
            trial_config['Secondary objects'].remove(trial_config['Target object'][0])
        return trial_config

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            img_recv = False
            try:
                orig_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
                cv_img = orig_img.copy()
                if msg.object_found:
                    box_2d = message_converter.convert_ros_message_to_dictionary(msg.box2d)
                    start_pt = (box_2d['min_x'], box_2d['min_y'])
                    end_pt = (box_2d['max_x'], box_2d['max_y'])
                    cv_img = cv2.rectangle(cv_img, start_pt, end_pt, (255, 0, 0), 2)
                    cv_img = cv2.resize(cv_img, (int(cv_img.shape[1] *0.5), int(cv_img.shape[0] * 0.5)))
                img_recv = True
            except CvBridgeError as e:
                print(e)

            if img_recv:
                self.result_widgets['Result'].setText('Complete')
                qimg = QtGui.QImage(cv_img.data, cv_img.shape[1], cv_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
                self.result_widgets['Image'].setPixmap(QtGui.QPixmap.fromImage(qimg))
            else:
                self.result_widgets['Result'].setText('Complete -- no image received')
            widget = self.result_widgets['Object found']
            if msg.object_found:
                widget.setText("True")
                if msg.result_type == ObjectDetectionResult.BOUNDING_BOX_2D:
                    self.result_widgets['Result type'].setText('2D bounding box')
                else:
                    self.result_widgets['Result type'].setText('3D bounding box')
            else:
                widget.setText("False")

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['object_found'] = msg.object_found
        if msg.result_type == ObjectDetectionResult.BOUNDING_BOX_2D:
            result['result_type'] = "2D bounding box"
            result['box2d'] = message_converter.convert_ros_message_to_dictionary(msg.box2d)
        else:
            result['result_type'] = "3D bounding box"
            result['box3d'] = message_converter.convert_ros_message_to_dictionary(msg.box3d)
        try:
            cv_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
            result['images'] = [cv_img]
        except CvBridgeError as e:
            result['images'] = None

        return result

    def get_task_info_for_robot(self):
        var = 'Target object'
        widget = self.variation_widgets[var]
        config = {}
        for child in widget.children():
            if isinstance(child, QRadioButton) or isinstance(child, QCheckBox):
                if child.isChecked():
                    config['Target object'] = child.text()
            if isinstance(child, QComboBox):
                config['Target object'] = child.currentText()
        return config


