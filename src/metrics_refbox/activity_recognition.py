from metrics_refbox.benchmark_config import BenchmarkConfig

class ActivityRecognitionConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(ActivityRecognitionConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

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
