import random
from python_qt_binding.QtWidgets import QCheckBox, QRadioButton, QComboBox
from benchmark_config import BenchmarkConfig
from metrics_refbox_msgs.msg import ObjectDetectionResult
from rospy_message_converter import message_converter

class ObjectDetectionConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(ObjectDetectionConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

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
            self.result_widgets['Result'].setText('Complete')
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


