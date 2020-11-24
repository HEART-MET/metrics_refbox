from metrics_refbox.benchmark_config import BenchmarkConfig

class HandoverObjectConfig(BenchmarkConfig):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        super(HandoverObjectConfig, self).__init__(config, config_path, benchmark_name, benchmark_result_type)

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

    def show_results(self, msg, timeout, stopped):
        if timeout:
            self.result_widgets['Result'].setText('Timeout')
        elif stopped:
            self.result_widgets['Result'].setText('Stopped')
        else:
            self.result_widgets['Result'].setText('Complete')
            self.result_widgets['Detected pose'].setText(self.pose_map[msg.human_pose])
            self.result_widgets['Detected pre-grasp behaviour'].setText(self.reach_map[msg.human_reach_out_result])
            self.result_widgets['Detected behaviour during grasp'].setText(self.grasp_map[msg.grasp_result])
            self.result_widgets['Detected post-grasp behaviour'].setText(self.post_grasp_map[msg.post_grasp_result])

    def get_result_dict_from_msg(self, msg):
        result = {}
        result['detected_human_pose'] = self.pose_map[msg.human_pose]
        result['detected_pre_grasp_behaviour'] = self.reach_map[msg.human_reach_out_result]
        result['detected_grasp_result'] = self.grasp_map[msg.grasp_result]
        result['detected_post_grasp_behaviour'] = self.post_grasp_map[msg.post_grasp_result]
        return result
