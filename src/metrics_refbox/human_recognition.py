from metrics_refbox.benchmark_config import BenchmarkConfig

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
            if len(msg.identities) > 0:
                widget.setText(msg.identities[0])

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['recognized_person'] = msg.identities
        return result

    def get_trial_result_dict(self, msg, current_trial_name, current_team_name, timeout, stopped, elapsed_time):
        results = super(HumanRecognitionConfig, self).get_trial_result_dict(msg, current_trial_name, current_team_name, timeout, stopped, elapsed_time)
        results['config']['Target Persons'] = self.config['Target Persons']
        return results
