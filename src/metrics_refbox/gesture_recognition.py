from metrics_refbox.benchmark_config import BenchmarkConfig

class GestureRecognitionConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(GestureRecognitionConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')
            widget = self.result_widgets['Recognized gesture']
            if len(msg.gestures) > 0:
                widget.setText(msg.gestures[0])

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['recognized_gesture'] = msg.gestures
        return result
