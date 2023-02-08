try:
    from metrics_refbox.benchmark_config import BenchmarkConfig
except:
    from benchmark_config import BenchmarkConfig

class GenericTaskConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(GenericTaskConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')
            widget = self.result_widgets['Response']
            widget.setText(msg.result_message)

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['response'] = msg.result_message
        return result
