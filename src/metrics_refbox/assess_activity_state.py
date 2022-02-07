from metrics_refbox.benchmark_config import BenchmarkConfig
from rospy_message_converter import message_converter
import datetime

class AssessActivityStateConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(AssessActivityStateConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

    def is_detected(self, box):
        if box.min_x == box.max_x or box.min_y == box.max_y:
            return False
        return True


    def show_feedback(self, msg):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        m = '[' + timestamp + '] '
        if msg.phase == msg.PHASE_DETECTION:
            img = msg.image
            box = msg.box2d
            if self.is_detected(box):
                m += ' detected person' + '\n'
        elif msg.phase == msg.PHASE_VISUAL_ASSESSMENT or msg.phase == msg.PHASE_VERBAL_ASSESSMENT:
            if len(msg.activities) > 0:
                m += 'visually ' if msg.phase == msg.PHASE_VISUAL_ASSESSMENT else 'verbally '
                m += 'assessed activities:\n'
                for act in msg.activities:
                    m += '\t' + act + '\n'
            else:
                m += 'no activities '
                m += 'visually ' if msg.phase == msg.PHASE_VISUAL_ASSESSMENT else 'verbally '
                m += 'determined\n'
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
            widget = self.result_widgets['Recognized activity']
            if len(msg.activities) > 0:
                widget.setText(msg.activities[0])

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['recognized_activities'] = msg.activities
        return result

    def get_feedback_dict_from_msgs(self, msgs, timestamps):
        feedback = []
        for idx, msg in enumerate(msgs):
            feedback_msg = {}
            if msg.phase == msg.PHASE_DETECTION:
                feedback_msg['phase'] = 'detection'
                feedback_msg['box2d'] = message_converter.convert_ros_message_to_dictionary(msg.box2d)
            elif msg.phase == msg.PHASE_VISUAL_ASSESSMENT:
                feedback_msg['phase'] = 'visual'
                feedback_msg['recognized_activities'] = msg.activities
            elif msg.phase == msg.PHASE_VERBAL_ASSESSMENT:
                feedback_msg['phase'] = 'verbal'
                feedback_msg['recognized_activities'] = msg.activities
            feedback.append(feedback_msg)
        return feedback
