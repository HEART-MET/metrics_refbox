from metrics_refbox.benchmark_config import BenchmarkConfig
import datetime

class ClutteredPickConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(ClutteredPickConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')

    def show_feedback(self, msg):
        if msg.action_completed == msg.PICKED:
            action = 'picked'
        elif msg.action_completed == msg.PLACED:
            action = 'placed'
        else:
            action = 'invalid action'
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        m = '[' + timestamp + '] ' + msg.object_name + ' ' + action + '\n'
        self.result_widgets['Feedback'].insertPlainText(m)
        self.result_widgets['Feedback'].ensureCursorVisible()

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['num_objects_picked'] = msg.num_objects_picked
        return result

    def get_feedback_dict_from_msgs(self, msgs, timestamps):
        feedback = []
        for idx, msg in enumerate(msgs):
            feedback_msg = {}
            feedback_msg['object_name'] = msg.object_name
            if msg.action_completed == msg.PICKED:
                feedback_msg['action'] = 'PICK'
            elif msg.action_completed == msg.PLACED:
                feedback_msg['action'] = 'PLACE'
            feedback_msg['time'] = timestamps[idx]
            feedback.append(feedback_msg)
        return feedback
