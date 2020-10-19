from benchmark_config import BenchmarkConfig

class HumanRecognitionConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(HumanRecognitionConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')
            widget = self.result_widgets['Recognized person']
            widget.setText(msg.identity)

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['recognized_person'] = msg.identity
        return result
