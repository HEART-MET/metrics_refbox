from python_qt_binding.QtWidgets import QVBoxLayout, QHBoxLayout, QLabel, QScrollArea, QCheckBox,\
            QWidget, QRadioButton, QGroupBox, QLineEdit, QPlainTextEdit, QMessageBox, QComboBox
from python_qt_binding.QtCore import Qt
import random
import numpy as np
import os
import json

class BenchmarkConfig(object):
    def __init__(self, config, config_path, benchmark_name, benchmark_result_type):
        self.config = config
        self.config_path = config_path
        self.benchmark_name = benchmark_name
        self.result_type = benchmark_result_type
        self.variation_widgets = {}
        self.result_widgets = {}
        self.bagfile_name = None
        self.notes_widget = None

    def setup(self, config_layout, results_layout, config_locked):
        '''
        set up the layout for selecting options from the different benchmark variations
        '''
        config_group_box_layout = QHBoxLayout()
        self.config_group_box = QGroupBox('Configuration')
        self.config_group_box.setCheckable(True)
        if (config_locked):
            self.config_group_box.setChecked(False)
        variations = self.config['variations'].keys()
        for var in variations:
            single_choice = True
            if var in self.config['multiple choice variation']:
                single_choice = False
            if isinstance(self.config['variations'][var], list):
                self.variation_widgets[var], scroll = self.add_config(var, self.config['variations'][var], single_choice=single_choice)
                config_group_box_layout.addWidget(scroll)
            else:
                path = os.path.join(self.config_path, self.config['variations'][var])
                if os.path.exists(path):
                    with open(path) as f:
                        options = f.readlines()
                    options = [x.strip() for x in options]
                    self.variation_widgets[var], scroll = self.add_config(var, options, single_choice=single_choice)
                    config_group_box_layout.addWidget(scroll)
                    self.config['variations'][var] = options
                else:
                    QMessageBox.critical(None, "Error", "File %s does not exist" % path)

        self.config_group_box.setLayout(config_group_box_layout)
        config_layout.addWidget(self.config_group_box)


        results_group_box = QGroupBox('Results')
        results_group_box_layout = QHBoxLayout()
        results = self.config['results'].keys()
        for res in results:
            single_choice = True
            if var in self.config['multiple choice result']:
                single_choice = False
            if isinstance(self.config['results'][res], list):
                self.result_widgets[res], scroll = self.add_config(res, self.config['results'][res], single_choice=single_choice)
                results_group_box_layout.addWidget(scroll)
            else:
                if self.config['results'][res] == 'text_field':
                    widget = QWidget()
                    text_field = QLineEdit()
                    text_field.setReadOnly(True)
                    txtlayout = QHBoxLayout()
                    txtlayout.addWidget(QLabel(res))
                    txtlayout.addWidget(text_field)
                    widget.setLayout(txtlayout)
                    self.result_widgets[res] = text_field
                    results_group_box_layout.addWidget(widget)
                elif self.config['results'][res] == 'text_area':
                    widget = QWidget()
                    text_area = QPlainTextEdit()
                    text_area.setReadOnly(True)
                    text_area.setMaximumHeight(200)
                    txtlayout = QHBoxLayout()
                    txtlayout.addWidget(QLabel(res))
                    txtlayout.addWidget(text_area)
                    widget.setLayout(txtlayout)
                    self.result_widgets[res] = text_area
                    results_group_box_layout.addWidget(widget)
                elif self.config['results'][res] == 'image':
                    img = QLabel()
                    self.result_widgets[res] = img
                    results_group_box_layout.addWidget(img)

        results_group_box.setLayout(results_group_box_layout)
        results_layout.addWidget(results_group_box)

        self.notes_widget = QPlainTextEdit()
        self.notes_widget.setMaximumHeight(100)
        self.notes_widget.setPlaceholderText('Enter notes about the result...')
        results_layout.addWidget(self.notes_widget)

    def add_config(self, title, choices, single_choice=True):
        '''
        create a UI element for selecting options for one variation
        and put it in a scrollArea
        '''
        scroll = QScrollArea()
        group_box = QGroupBox(title)
        group_box.setFlat(True)

        layout = QVBoxLayout()
        if len(choices) > 5 and single_choice:
            combo_box = QComboBox(group_box)
            for obj in choices:
                combo_box.addItem(obj)
            layout.addWidget(combo_box)
        else:
            for obj in choices:
                if single_choice:
                    layout.addWidget(QRadioButton(obj, group_box))
                else:
                    layout.addWidget(QCheckBox(obj, group_box))
        layout.addStretch(1)

        group_box.setLayout(layout)
        scroll.setWidget(group_box)
        scroll.setWidgetResizable(True)
        return group_box, scroll

    def get_current_selections(self):
        '''
        read current selections from the config UI elements
        and return it as a dictionary
        '''
        variations = self.config['variations'].keys()
        trial_config = {}
        for var in variations:
            widget = self.variation_widgets[var]
            selections = []

            for child in widget.children():
                if isinstance(child, QRadioButton) or isinstance(child, QCheckBox):
                    if child.isChecked():
                        selections.append(child.text())
                elif isinstance(child, QComboBox):
                    selections.append(child.currentText())

            trial_config[var] = selections
        return trial_config

    def apply_selections(self, trial_config):
        '''
        apply selections specified in trial_config to config UI elements
        '''
        variations = self.config['variations'].keys()
        for var in variations:
            widget = self.variation_widgets[var]
            selections = trial_config[var]
            if len(selections) == 0:
                return
            for child in widget.children():
                if isinstance(child, QRadioButton) or isinstance(child, QCheckBox):
                    if child.text() in selections:
                        child.setChecked(True)
                    else:
                        child.setChecked(False)
                elif isinstance(child, QComboBox):
                    child.setCurrentText(selections[0])

    def clear_selections(self):
        '''
        remove all selections in config UI elements
        (except radio buttons since they cannot be unselected)
        '''
        variations = self.config['variations'].keys()
        for var in variations:
            widget = self.variation_widgets[var]
            for child in widget.children():
                if isinstance(child, QRadioButton) or isinstance(child, QCheckBox):
                    child.setChecked(False)
                elif isinstance(child, QComboBox):
                    child.setCurrentIndex(0)

    def generate(self):
        '''
        Generate a set of selections for all config elements
        '''
        trial_config = {}
        variations = self.config['variations'].keys()
        for var in variations:
            single_choice = True
            if var in self.config['multiple choice variation']:
                single_choice = False

            selected_indices = []
            if single_choice:
                choices = list(range(len(self.config['variations'][var])))
                probabilities = None
                if var in self.config['selection likelihood'].keys():
                    probabilities = self.config['selection likelihood'][var]
                choice = np.random.choice(choices, p=probabilities)
                selected_indices.append(choice)
            else:
                num_choices = len(self.config['variations'][var])
                # TODO: should this be configurable?
                min_selections = 2
                max_selections = 5
                num_selections = random.randint(min_selections, max_selections)
                for idx in range(num_selections):
                    choice = random.randrange(0, num_choices)
                    selected_indices.append(choice)
            selected_items = np.array(self.config['variations'][var])[selected_indices].tolist()
            trial_config[var] = selected_items
        return trial_config

    def show_results(self, msg, timeout, stopped):
        pass

    def clear_results(self):
        '''
        clear results elements
        '''
        self.bagfile_name = None
        results = self.config['results'].keys()
        for res in results:
            if isinstance(self.config['results'][res], list):
                single_choice = True
                if var in self.config['multiple choice result']:
                    single_choice = False
                if not single_choice:
                    widget = self.result_widgets[res]
                    for child in widget.children():
                        if isinstance(child, QRadioButton) or isinstance(child, QCheckBox):
                            child.setChecked(False)
                        elif isinstance(child, QComboBox):
                            child.setCurrentIndex(0)
            elif self.config['results'][res] == 'text_field':
                self.result_widgets[res].setText('')
            elif self.config['results'][res] == 'text_area':
                self.result_widgets[res].clear()
        self.notes_widget.clear()


    def get_trial_result_dict(self, msg, feedback_msgs, feedback_timestamps, current_trial_name, current_team_name, timeout, stopped, elapsed_time):
        '''
        return dictionary with all result fields for this benchmark
        '''
        results = {}
        results['trial_id'] = current_trial_name
        results['team_name'] = current_team_name
        results['bagfile'] = self.bagfile_name
        results['duration'] = elapsed_time
        results['timeout'] = timeout
        results['stopped'] = stopped
        results['config'] = self.get_current_selections()
        results['notes'] = self.notes_widget.toPlainText()
        if not stopped and not timeout:
            results['results'] = self.get_result_dict_from_msg(msg)
        else:
            results['results'] = {}
        if feedback_msgs:
            results['feedback'] = self.get_feedback_dict_from_msgs(feedback_msgs, feedback_timestamps)
        else:
            results['feedback'] = {}

        return results

    def get_result_dict_from_msg(self, msg):
        '''
        Return benchmark-specific results in the form of a dictionary
        Must be overridden by subclasses.
        '''
        pass

    def get_feedback_dict_from_msgs(self, msgs):
        '''
        Return benchmark-specific feedback in the form of a dictionary
        Must be overridden by subclasses.
        '''
        pass

    def get_task_info_for_robot(self):
        '''
        return config information which is required for the robot to perform the task
        e.g. object to be detected, grasp pose, navigation poses etc.
        '''
        return {}

    def get_timeout(self):
        return self.config["timeout"]

    def set_bagfile_name(self, name):
        self.bagfile_name = name

    def get_bagfile_name(self):
        return self.bagfile_name

    def lock_config(self):
        self.config_group_box.setChecked(False)

    def unlock_config(self):
        self.config_group_box.setChecked(True)

