
from python_qt_binding.QtWidgets import QApplication, QLabel, QWidget, QGroupBox,\
                    QVBoxLayout, QGridLayout, QPushButton, QComboBox, \
                    QListWidget, QListWidgetItem, QScrollArea, QCheckBox, QMessageBox, \
                    QAbstractItemView, QPlainTextEdit, QHBoxLayout, QLineEdit, QInputDialog \

from python_qt_binding.QtCore import Qt, pyqtSignal, QElapsedTimer, QTimer
from python_qt_binding.QtGui import QFont
from . import metrics_refbox
import metrics_refbox_msgs.msg
import json
import sys
import datetime
import glob
import os
import importlib
import cv2


SIDEBAR_WIDTH = 200
class MetricsRefboxWidget(QWidget):

    status_signal = pyqtSignal(object)
    timeout_signal = pyqtSignal(object)
    result_signal = pyqtSignal(object)
    feedback_signal = pyqtSignal(object)
    bagfile_signal = pyqtSignal(object)


    def __init__(self):
        super(MetricsRefboxWidget, self).__init__()
        self.status_signal.connect(self.update_status)
        self.timeout_signal.connect(self._handle_timeout)
        self.result_signal.connect(self.update_result)
        self.feedback_signal.connect(self.update_feedback)
        self.bagfile_signal.connect(self._update_bagfile_name)
        self.timer = QElapsedTimer()
        self.timer_interrupt = QTimer(self)
        self.timer_interrupt.timeout.connect(self.update_timer)
        self.timer_interrupt.start(100)

        self.metrics_refbox = metrics_refbox.MetricsRefbox(self._status_cb, self._start_cb, self._result_cb)
        self.trial_configs = {}
        self.current_benchmark = None
        self.current_benchmark_enum = -1
        # once we receive confirmation from robot
        self.benchmark_running = False
        # once we send the start message to robot
        self.benchmark_started = False
        # set to True if we're not recording individual trials
        # but rather continuously recording all data (hence no timeouts)
        self.continuous_recording = False
        self.timeout = False
        self.stopped = False
        self.result_msg = None
        self.feedback_msgs = None
        self.feedback_timestamps = None
        self.config_locked = True
        self.config = self.metrics_refbox.get_config()
        self.setup()
        self.show()

    def setup(self):
        layout = QGridLayout()

        # Sidebar
        self.benchmark_combo_box = QComboBox()
        for key in self.config['benchmarks'].keys():
            self.benchmark_combo_box.addItem(self.config['benchmarks'][key]['name'])
        self.benchmark_combo_box.currentIndexChanged.connect(self._handle_benchmark_selected)
        self.benchmark_combo_box.setMaximumWidth(SIDEBAR_WIDTH)
        self.set_current_benchmark()

        self.team_combo_box = QComboBox()
        for key in self.config['teams']:
            self.team_combo_box.addItem(key)
        self.team_combo_box.setMaximumWidth(SIDEBAR_WIDTH)
        self.test_communication_button = QPushButton('Test communication')
        self.test_communication_button.clicked.connect(self._handle_test_comm)

        self.trial_list_widget = QListWidget()
        self.trial_list_widget.currentItemChanged.connect(self._handle_trial_change)
        self.trial_list_widget.setMaximumWidth(SIDEBAR_WIDTH)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.addWidget(QLabel("Team"))
        sidebar_layout.addWidget(self.team_combo_box)
        sidebar_layout.addWidget(self.test_communication_button)
        sidebar_layout.addWidget(QLabel("Benchmark"))
        sidebar_layout.addWidget(self.benchmark_combo_box)
        sidebar_layout.addWidget(QLabel("Trial"))
        sidebar_layout.addWidget(self.trial_list_widget)

        self.generate_button = QPushButton('Generate')
        self.generate_button.clicked.connect(self._handle_generate)

        self.delete_button = QPushButton('Delete')
        self.delete_button.clicked.connect(self._handle_delete)

        self.save_trials_button = QPushButton('Save')
        self.save_trials_button.clicked.connect(self._handle_save_trial_config)

        self.lock_button = QPushButton('Lock')
        if self.config_locked:
            self.lock_button.setText('Unlock')
        self.lock_button.clicked.connect(self._handle_lock)

        sidebar_button_layout = QGridLayout()
        sidebar_button_layout.addWidget(self.generate_button, 0, 0)
        sidebar_button_layout.addWidget(self.delete_button, 0, 1)
        sidebar_button_layout.addWidget(self.save_trials_button, 1, 0)
        sidebar_button_layout.addWidget(self.lock_button, 1, 1)
        sidebar_layout.addLayout(sidebar_button_layout)

        layout.addLayout(sidebar_layout, 0, 0)

        # Status box
        self.status = QPlainTextEdit()
        self.status.setReadOnly(True)
        self.status.setMaximumHeight(200)

        # row 1, col 0, rowspan 1, colspan 2
        layout.addWidget(self.status, 1, 0, 1, 2)

        # trial config and results
        trial_layout = QVBoxLayout()
        self.trial_config_layout = QHBoxLayout()
        self.trial_results_layout = QVBoxLayout()
        self.setup_trial_config()


        # benchmark trial controls
        benchmark_controls_group_box = QGroupBox('Controls')
        benchmark_controls_layout = QHBoxLayout()
        self.start_trial_button = QPushButton('Start')
        self.start_trial_button.clicked.connect(self._handle_start_trial)
        self.stop_trial_button = QPushButton('Stop')
        self.stop_trial_button.clicked.connect(self._handle_stop_trial)
        self.prev_trial_button = QPushButton('Previous')
        self.prev_trial_button.clicked.connect(self._handle_prev_trial)
        self.next_trial_button = QPushButton('Next')
        self.next_trial_button.clicked.connect(self._handle_next_trial)
        self.start_continuous_recording_button = QPushButton('Start continuous recording')
        self.start_continuous_recording_button.clicked.connect(self._handle_continuous_recording)

        self.timer_field = QLabel()
        font = QFont("Arial", 20, QFont.Bold)
        self.timer_field.setFont(font)
        self.timer_field.setAutoFillBackground(True)
        benchmark_controls_layout.addWidget(self.start_trial_button)
        benchmark_controls_layout.addWidget(self.stop_trial_button)
        benchmark_controls_layout.addWidget(self.prev_trial_button)
        benchmark_controls_layout.addWidget(self.next_trial_button)
        benchmark_controls_layout.addWidget(self.start_continuous_recording_button)
        benchmark_controls_layout.addWidget(self.timer_field)
        benchmark_controls_group_box.setLayout(benchmark_controls_layout)

        trial_layout.addLayout(self.trial_config_layout)
        trial_layout.addWidget(benchmark_controls_group_box)
        trial_layout.addLayout(self.trial_results_layout)
        self.save_results_button = QPushButton('Save results')
        self.save_results_button.clicked.connect(self._handle_save_result)
        trial_layout.addWidget(self.save_results_button)

        layout.addLayout(trial_layout, 0, 1)

        self.setLayout(layout)
        self.show()

        self.show_env_var('ROS_MASTER_URI')
        self.show_env_var('ROS_IP')
        self.show_env_var('ROS_HOSTNAME')

    def show_env_var(self, var):
        env_str = os.getenv(var) if os.getenv(var) is not None else ''
        msg = var + ': ' + env_str + '\n'
        self.update_status(msg)

    def set_current_benchmark(self):
        '''
        Sets the current benchmark type based on user selection
        Load config file for selected benchmark and setup GUI to show config and results based on that
        '''
        benchmark_index = self.benchmark_combo_box.currentIndex()
        benchmark_name = list(self.config['benchmarks'].keys())[benchmark_index]
        benchmark_result_type = getattr(metrics_refbox_msgs.msg, self.config['benchmarks'][benchmark_name]['type'])
        config_file_name = benchmark_name + '.json'
        config_file = os.path.join(self.metrics_refbox.get_config_file_path(), config_file_name)
        stream = open(config_file, 'r')
        benchmark_config = json.load(stream)
        module = importlib.import_module(benchmark_config['module'])
        self.current_benchmark = getattr(module, benchmark_config['class']) (benchmark_config, self.metrics_refbox.get_config_file_path(), benchmark_name, benchmark_result_type)
        self.current_benchmark_enum = self.config['benchmarks'][benchmark_name]['enum']

    def setup_trial_config(self):
        '''
        set up the benchmark specific part of the GUI related to the configuration of
        the benchmark (for example, which objects will be used for object detection)

        the specific configuration is loaded based on the selected trial_index
        '''


        for i in reversed(range(self.trial_config_layout.count())):
            self.trial_config_layout.itemAt(i).widget().setParent(None)

        for i in reversed(range(self.trial_results_layout.count())):
            self.trial_results_layout.itemAt(i).widget().setParent(None)

        self.current_benchmark.setup(self.trial_config_layout, self.trial_results_layout, self.config_locked)

        self.trial_list_widget.clear()
        self.trial_configs.clear()

        path = os.path.join(self.metrics_refbox.get_config_file_path(), 'trials')
        trial_config_file = os.path.join(path, self.current_benchmark.config['trial_config'])
        if os.path.exists(trial_config_file):
            with open(trial_config_file, "r") as fp:
                trial_config = json.load(fp)
            for key in sorted(trial_config.keys()):
                trial_item = QListWidgetItem(key)
                trial_item.setFlags(trial_item.flags() | Qt.ItemIsEditable)
                self.trial_list_widget.addItem(trial_item)
                self.trial_configs[key] = trial_config[key]
            self.trial_list_widget.setCurrentRow(0)

    def _handle_test_comm(self):
        '''
        'Test communication' button
        '''
        self.metrics_refbox.test_communication()
        self.status_signal.emit('Sent test message to all clients\n')

    def _handle_benchmark_selected(self, idx):
        '''
        benchmark selection has changed
        '''
        self._handle_save_trial_config(None)
        self.set_current_benchmark()
        self.setup_trial_config()

    def _handle_trial_change(self, current_item, previous_item):
        '''
        Trial selection has changed
        '''
        if previous_item is not None:
            self.trial_configs[previous_item.text()] = self.current_benchmark.get_current_selections()
            self.current_benchmark.clear_results()
        if current_item is not None and current_item.text() in self.trial_configs.keys() and self.trial_configs[current_item.text()] is not None:
            self.current_benchmark.apply_selections(self.trial_configs[current_item.text()])
        else:
            self.current_benchmark.clear_selections()

    def _handle_generate(self, button):
        '''
        generate a new trial config
        '''
        trial_config = self.current_benchmark.generate()
        names = []
        for key in self.trial_configs.keys():
            names.append(key)
        msg = 'Select a name for this trial configuration'
        text = ''
        while True:
            text, ok = QInputDialog.getText(self, 'Trial name', msg)
            if text not in names:
                break
            QMessageBox.critical(self, "Error", "Name exists already")
        if ok:
            trial_item = QListWidgetItem(text)
            trial_item.setFlags(trial_item.flags() | Qt.ItemIsEditable)
            self.trial_list_widget.addItem(trial_item)
            self.trial_configs[text] = trial_config
            self.trial_list_widget.setCurrentItem(trial_item)

    def _handle_delete(self, button):
        '''
        delete current trial config
        '''
        selected_items = self.trial_list_widget.selectedItems()
        for item in selected_items:
            self.trial_list_widget.takeItem(self.trial_list_widget.row(item))
            del self.trial_configs[item.text()]


    def _handle_save_trial_config(self, button):
        '''
        save trial config in case it has been edited
        '''
        if self.trial_list_widget.currentItem() is not None:
            self.trial_configs[self.trial_list_widget.currentItem().text()] = self.current_benchmark.get_current_selections()
        path = os.path.join(self.metrics_refbox.get_config_file_path(), 'trials')
        trial_config_file = os.path.join(path, self.current_benchmark.config['trial_config'])
        with open(trial_config_file, "w") as fp:
            json.dump(self.trial_configs, fp)

    def _handle_lock(self, button):
        '''
        Lock trial config settings so they are not changed accidentally
        '''
        if self.config_locked:
            self.current_benchmark.unlock_config()
            self.config_locked = False
            self.lock_button.setText('Lock')
        else:
            self.current_benchmark.lock_config()
            self.config_locked = True
            self.lock_button.setText('Unlock')


    def _handle_start_trial(self, button):
        '''
        Trial start button; send command to refbox client via ROS node
        '''
        if self.benchmark_running or self.benchmark_started:
            self.update_status("Benchmark has already started. Stop and start the benchmark if you want to restart it\n")
        else:
            self.metrics_refbox.start(self.current_benchmark_enum, self.current_benchmark.get_task_info_for_robot())
            self.benchmark_started = True
            self.current_benchmark.clear_results()
            self.update_status("Sent start command\n")
            self.timer_field.setText('')
            self.disable_buttons()

    def _handle_stop_trial(self, button):
        '''
        Trial stop button
        '''
        self.metrics_refbox.stop()
        self.elapsed_time = self.timer.elapsed()
        self.stopped = True
        self.benchmark_running = False
        self.benchmark_started = False
        self.status_signal.emit("Sent stop command\n")
        self.result_signal.emit(None)
        self.enable_buttons()

    def _handle_next_trial(self, button):
        '''
        Select next trial
        '''
        row = self.trial_list_widget.currentRow()
        if (row + 1 < self.trial_list_widget.count()):
            self.trial_list_widget.setCurrentRow(row + 1)

    def _handle_prev_trial(self, button):
        '''
        Select previous trial
        '''
        row = self.trial_list_widget.currentRow()
        if (row - 1 >= 0):
            self.trial_list_widget.setCurrentRow(row - 1)

    def _handle_continuous_recording(self, button):
        '''
        Start recording button, not tied to any benchmark, and requires user to stop recording
        '''
        if self.benchmark_running or self.benchmark_started:
            self.metrics_refbox.stop()
            self.elapsed_time = self.timer.elapsed()
            self.stopped = True
            self.benchmark_running = False
            self.benchmark_started = False
            self.status_signal.emit("Sent stop command\n")
            self.start_continuous_recording_button.setText('Start continuous recording')
            self.enable_buttons()
            self.stop_trial_button.setEnabled(True)
            self.continuous_recording = False
        else:
            self.metrics_refbox.start_recording()
            self.continuous_recording = True
            self.start_continuous_recording_button.setText('Stop recording')
            self.benchmark_started = True
            self.update_status("Sent start command\n")
            self.timer_field.setText('')
            self.disable_buttons()
            self.stop_trial_button.setEnabled(False)


    def _handle_save_result(self, button):
        '''
        Save results again in case notes were added
        '''
        self.result_signal.emit(self.result_msg)

    def _start_cb(self, msg):
        '''
        called when we get confirmation that the robot received the start message;
        official start of trial time
        '''
        if msg.data == True:
            self.elapsed_time = 0.0
            self.timer.start()
            self.benchmark_running = True
            self.timeout = False
            self.stopped = False
            self.result_msg = None
            self.feedback_msgs = None
            self.feedback_timestamps = None
            if msg.rosbag_filename == '':
                self.status_signal.emit('Error: rosbag filename not specified although start message confirmed\n')
                self._handle_stop_trial(None)
                return
            self.status_signal.emit('Started trial and recording to %s\n' % msg.rosbag_filename)
            self.bagfile_signal.emit(msg.rosbag_filename)
        else:
            self.status_signal.emit('Error: Benchmark did not start, probably because the rosbag recorder is not running\n')
            self._handle_stop_trial(None)

    def _result_cb(self, msg):
        '''
        called when we get a result from the robot
        official end of trial time
        '''
        if self.benchmark_running:
            if msg.message_type == msg.RESULT:
                self.elapsed_time = self.timer.elapsed()
                self.benchmark_running = False
                self.benchmark_started = False

                self.status_signal.emit("Trial completed in %.2f seconds\n" % (self.elapsed_time / 1000.0))
                self.result_msg = msg
                self.result_signal.emit(msg)
                self.enable_buttons()
            elif msg.message_type == msg.FEEDBACK:
                if not self.feedback_msgs:
                    self.feedback_msgs = []
                    self.feedback_timestamps = []
                self.feedback_msgs.append(msg)
                self.feedback_timestamps.append(self.timer.elapsed() / 1000.0)
                self.feedback_signal.emit(msg)
            else:
                self.status_signal.emit("Invalid message type received from robot")

    def _status_cb(self, msg):
        '''
        callback to send text to status box
        '''
        self.status_signal.emit(msg)

    def update_status(self, msg):
        '''
        signal handler for status box
        '''
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        m = '[' + timestamp + '] ' + msg
        self.status.insertPlainText(m)
        self.status.ensureCursorVisible();

    def _update_bagfile_name(self, name):
        self.current_benchmark.set_bagfile_name(name)

    def update_result(self, msg):
        '''
        signal handler for result message; process and save result
        '''
        self.current_benchmark.show_results(msg, self.timeout, self.stopped)
        current_trial_name = self.trial_list_widget.currentItem().text()
        current_team_name = self.team_combo_box.currentText()
        results_dict = self.current_benchmark.get_trial_result_dict(msg, self.feedback_msgs, self.feedback_timestamps, current_trial_name, current_team_name,
                                    self.timeout, self.stopped, self.elapsed_time / 1000.0)

        filename = self.current_benchmark.get_bagfile_name()[:-4] + '_' + self.current_benchmark.benchmark_name + '.json'
        path = os.path.join(self.metrics_refbox.get_results_file_path(), filename)
        if 'images' in results_dict['results'].keys():
            images = results_dict['results'].pop('images', None)
            if images is not None:
                img_folder = path[:-5] # folder with same name as json file
                if not os.path.exists(img_folder):
                    os.mkdir(img_folder)
                for idx, img in enumerate(images):
                    cv2.imwrite(os.path.join(img_folder, 'frame_%04d.jpg' % idx), img)
        if 'feedback' in results_dict.keys():
            for feedback in results_dict['feedback']:
                if 'images' in feedback.keys():
                    images = feedback.pop('images', None)
                    if images is not None:
                        img_folder = path[:-5] # folder with same name as json file
                        if not os.path.exists(img_folder):
                            os.mkdir(img_folder)
                        for idx, img in enumerate(images):
                            cv2.imwrite(os.path.join(img_folder, '%s_frame_%04d.jpg' % (feedback['phase'], idx)), img)

        with open(path, "w") as fp:
            json.dump(results_dict, fp)

    def update_feedback(self, msg):
        '''
        signal handler for feedback message; only display it
        '''
        self.current_benchmark.show_feedback(msg)

    def update_timer(self):
        '''
        Timer callback; update GUI, and also check if timeout has expired
        '''
        if not self.benchmark_running:
            return
        self.timer_field.setText("Time: %.1f s" % (self.timer.elapsed() / 1000.0))
        # if we're in continuous recording mode, no need to check timeout
        if self.continuous_recording:
            return
        if self.timer.hasExpired(self.current_benchmark.get_timeout() * 1000.0):
            self.status_signal.emit("Trial timeout of %.2f seconds has expired\n" % self.current_benchmark.get_timeout())
            self.timeout_signal.emit('timeout expired')

    def _handle_timeout(self, msg):
        '''
        timeout has expired, so stop the trial
        '''
        self.timeout = True
        self._handle_stop_trial(None)

    def closeEvent(self, event):
        '''
        make sure we save trial configs when window is closed
        '''
        self.metrics_refbox.stop()
        self._handle_save_trial_config(None)
        event.accept()

    def disable_buttons(self):
        '''
        disable buttons when trial is running
        '''
        self.team_combo_box.setEnabled(False)
        self.benchmark_combo_box.setEnabled(False)
        self.trial_list_widget.setEnabled(False)
        self.next_trial_button.setEnabled(False)
        self.prev_trial_button.setEnabled(False)
        self.delete_button.setEnabled(False)
        self.generate_button.setEnabled(False)
        self.save_trials_button.setEnabled(False)
        self.lock_button.setEnabled(False)
        self.start_trial_button.setEnabled(False)

    def enable_buttons(self):
        self.team_combo_box.setEnabled(True)
        self.benchmark_combo_box.setEnabled(True)
        self.trial_list_widget.setEnabled(True)
        self.next_trial_button.setEnabled(True)
        self.prev_trial_button.setEnabled(True)
        self.delete_button.setEnabled(True)
        self.generate_button.setEnabled(True)
        self.save_trials_button.setEnabled(True)
        self.lock_button.setEnabled(True)
        self.start_trial_button.setEnabled(True)


def main():
    app = QApplication(["Metrics Refbox"])
    #['Breeze', 'Windows', 'GTK+', 'Fusion']
    app.setStyle('Fusion')
    trw = MetricsRefboxWidget()
    app.exec_()
