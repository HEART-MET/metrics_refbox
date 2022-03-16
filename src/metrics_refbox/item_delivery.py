from metrics_refbox.benchmark_config import BenchmarkConfig
from rospy_message_converter import message_converter
import datetime
import cv2
from python_qt_binding import QtGui
from cv_bridge import CvBridge, CvBridgeError

class ItemDeliveryConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(ItemDeliveryConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)
        self.cv_bridge = CvBridge()
        self.pose_map = {}
        self.pose_map[0] = 'None'
        self.pose_map[self.result_type.HUMAN_POSE_LAYING] = 'Laying'
        self.pose_map[self.result_type.HUMAN_POSE_STANDING] = 'Standing'
        self.pose_map[self.result_type.HUMAN_POSE_SITTING] = 'Sitting'

        self.reach_map = {}
        self.reach_map[0] = 'None'
        self.reach_map[self.result_type.HUMAN_REACHED_OUT] = 'Reaches out'
        self.reach_map[self.result_type.HUMAN_DID_NOT_REACH_OUT] = 'Does not reach out'

        self.grasp_map = {}
        self.grasp_map[0] = 'None'
        self.grasp_map[self.result_type.GRASP_SUCCESSFUL] = 'Grasps object'
        self.grasp_map[self.result_type.GRASP_UNSUCCESSFUL] = 'Does not grasp object'

        self.post_grasp_map = {}
        self.post_grasp_map[0] = 'None'
        self.post_grasp_map[self.result_type.OBJECT_DROPPED_AFTER_GRASP] = 'Drops object'
        self.post_grasp_map[self.result_type.OBJECT_NOT_DROPPED_AFTER_GRASP] = 'Keeps object in hand'

    def is_detected(self, box):
        if box.min_x == box.max_x or box.min_y == box.max_y:
            return False
        return True

    def show_feedback(self, msg):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        m = '[' + timestamp + '] '
        if msg.phase == msg.PHASE_NAV_TO_OBJECT:
            m += ' completed navigating to object location' + '\n'
        elif msg.phase == msg.PHASE_OBJECT_DETECTION:
            img = msg.image
            box = msg.box2d
            if self.is_detected(box):
                m += ' detected object' + '\n'

            img_recv = False
            try:
                orig_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
                cv_img = orig_img.copy()
                if self.is_detected(box):
                    box_2d = message_converter.convert_ros_message_to_dictionary(msg.box2d)
                    start_pt = (box_2d['min_x'], box_2d['min_y'])
                    end_pt = (box_2d['max_x'], box_2d['max_y'])
                    cv_img = cv2.rectangle(cv_img, start_pt, end_pt, (255, 0, 0), 2)
                    cv_img = cv2.resize(cv_img, (int(cv_img.shape[1] *0.5), int(cv_img.shape[0] * 0.5)))
                img_recv = True
            except CvBridgeError as e:
                print(e)

            if img_recv:
                qimg = QtGui.QImage(cv_img.data, cv_img.shape[1], cv_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
                self.result_widgets['Image'].setPixmap(QtGui.QPixmap.fromImage(qimg))
        elif msg.phase == msg.PHASE_OBJECT_PICK:
            m += ' picked object' + '\n'
        elif msg.phase == msg.PHASE_NAV_TO_PERSON:
            m += " completed navigating to person's location" + '\n'
        elif msg.phase == msg.PHASE_PERSON_DETECTION:
            img = msg.image
            box = msg.box2d
            if self.is_detected(box):
                m += ' detected person' + '\n'

            img_recv = False
            try:
                orig_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
                cv_img = orig_img.copy()
                if self.is_detected(box):
                    box_2d = message_converter.convert_ros_message_to_dictionary(msg.box2d)
                    start_pt = (box_2d['min_x'], box_2d['min_y'])
                    end_pt = (box_2d['max_x'], box_2d['max_y'])
                    cv_img = cv2.rectangle(cv_img, start_pt, end_pt, (255, 0, 0), 2)
                    cv_img = cv2.resize(cv_img, (int(cv_img.shape[1] *0.5), int(cv_img.shape[0] * 0.5)))
                img_recv = True
            except CvBridgeError as e:
                print(e)

            if img_recv:
                qimg = QtGui.QImage(cv_img.data, cv_img.shape[1], cv_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
                self.result_widgets['Image'].setPixmap(QtGui.QPixmap.fromImage(qimg))

        elif msg.phase == msg.PHASE_HANDOVER:
            m += ' handover result: '
            if msg.human_pose == msg.HUMAN_POSE_LAYING:
                m += ' person is laying down, '
            elif msg.human_pose == msg.HUMAN_POSE_STANDING:
                m += ' person is standing, '
            elif msg.human_pose == msg.HUMAN_POSE_SITING:
                m += ' person is sitting, '
            else:
                m += ' person has unknown pose, '

            if msg.human_reach_out_result == msg.HUMAN_REACHED_OUT:
                m += ' person reached out, '
                if msg.grasp_result == msg.GRASP_SUCCESSFUL:
                    m += ' person grasped object, '
                    if msg.post_grasp_result == msg.OBJECT_NOT_DROPPED_AFTER_GRASP:
                        m += ' person did not drop object' + '\n'
                    elif msg.post_grasp_result == msg.OBJECT_DROPPED_AFTER_GRASP:
                        m += ' person dropped object' + '\n'
                    else:
                        m += ' unknown post-grasp result' + '\n'
                elif msg.grasp_result == msg.GRASP_UNSUCCESSFUL:
                    m += ' person did not grasp object\n'
                else:
                    m += ' unknown grasp result\n'
            elif msg.human_reach_out_result == msg.HUMAN_DID_NOT_REACH_OUT:
                m += ' person did not reach out\n'
            else:
                m += ' unknown result\n'
        else:
            m += 'invalid feedback\n'
        self.result_widgets['Feedback'].insertPlainText(m)
        self.result_widgets['Feedback'].ensureCursorVisible()

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')
            if msg.result == msg.RESULT_SUCCESS:
                self.result_widgets['Overall result'].setText('Success')
            elif msg.result == msg.RESULT_FAILURE:
                self.result_widgets['Overall result'].setText('Failure')

    def get_result_dict_from_msg(self, msg):
        result = {}
        if msg.result == msg.RESULT_SUCCESS:
            result['overall_result'] = 'Success'
        elif msg.result == msg.RESULT_FAILURE:
            result['overall_result'] = 'Failure'
        return result

    def get_feedback_dict_from_msgs(self, msgs, timestamps):
        feedback = []
        for idx, msg in enumerate(msgs):
            feedback_msg = {}
            if msg.phase == msg.PHASE_NAV_TO_OBJECT:
                feedback_msg['phase'] = 'nav_to_object'
            elif msg.phase == msg.PHASE_OBJECT_DETECTION:
                feedback_msg['phase'] = 'object_detection'
                feedback_msg['box2d'] = message_converter.convert_ros_message_to_dictionary(msg.box2d)
                try:
                    cv_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
                    feedback_msg['images'] = [cv_img]
                except CvBridgeError as e:
                    feedback_msg['images'] = None
            elif msg.phase == msg.PHASE_OBJECT_PICK:
                feedback_msg['phase'] = 'object_pick'
            elif msg.phase == msg.PHASE_NAV_TO_PERSON:
                feedback_msg['phase'] = 'nav_to_person'
            elif msg.phase == msg.PHASE_PERSON_DETECTION:
                feedback_msg['phase'] = 'person_detection'
                feedback_msg['box2d'] = message_converter.convert_ros_message_to_dictionary(msg.box2d)
                try:
                    cv_img = self.cv_bridge.imgmsg_to_cv2(msg.image, "passthrough")
                    feedback_msg['images'] = [cv_img]
                except CvBridgeError as e:
                    feedback_msg['images'] = None
            elif msg.phase == msg.PHASE_HANDOVER:
                feedback_msg['phase'] = 'handover'
                feedback_msg['detected_human_pose'] = self.pose_map[msg.human_pose]
                feedback_msg['detected_pre_grasp_behaviour'] = self.reach_map[msg.human_reach_out_result]
                feedback_msg['detected_grasp_result'] = self.grasp_map[msg.grasp_result]
                feedback_msg['detected_post_grasp_behaviour'] = self.post_grasp_map[msg.post_grasp_result]
            feedback.append(feedback_msg)
        return feedback
