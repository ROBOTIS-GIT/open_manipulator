#!/usr/bin/env python

# Standard Python imports
import argparse
import math
import random
import signal
import sys
from threading import Thread
import time
import xml.dom.minidom

# ROS 2 imports
import rclpy
import rclpy.parameter
import sensor_msgs.msg

# Python QT Binding imports
from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QSpinBox
from python_qt_binding.QtWidgets import QWidget

RANGE = 10000


class JointStatePublisher():
    def get_param(self, name, value=None):
        param = self.node.get_parameter(name)
        if param.type_ == rclpy.parameter.Parameter.Type.NOT_SET:
            return value
        return param.value

    def init_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    self.node.get_logger().warn("Unknown joint type %s", child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {'min':minval*math.pi/180.0, 'max':maxval*math.pi/180.0, 'zero':0, 'position':0, 'velocity':0, 'effort':0}
                self.free_joints[name] = joint

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -math.pi
                    maxval = math.pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        self.node.get_logger().warn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def __init__(self, node, description):
        self.node = node
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = self.get_param('dependent_joints', {})
        self.use_mimic = self.get_param('use_mimic_tags', True)
        self.use_small = self.get_param('use_smallest_joint_limits', True)

        self.zeros = self.get_param('zeros')

        self.pub_def_positions = self.get_param('publish_default_positions', True)
        self.pub_def_vels = self.get_param('publish_default_velocities', False)
        self.pub_def_efforts = self.get_param('publish_default_efforts', False)

        robot = xml.dom.minidom.parseString(description)
        if robot.getElementsByTagName('COLLADA'):
            self.init_collada(robot)
        else:
            self.init_urdf(robot)

        use_gui = self.get_param('use_gui', False)

        if use_gui:
            num_rows = self.get_param('num_rows', 0)
            self.app = QApplication(sys.argv)
            self.gui = JointStatePublisherGui("Joint State Publisher", self, num_rows)
            self.gui.show()
        else:
            self.gui = None

        source_list = self.get_param('source_list', [])
        self.sources = []
        for source in source_list:
            self.sources.append(self.node.create_subscription(sensor_msgs.msg.JointState, source, self.source_cb))

        self.pub = self.node.create_publisher(sensor_msgs.msg.JointState, 'joint_states')

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.gui is not None:
            # Signal instead of directly calling the update_sliders method, to
            # switch to the QThread.  Note that we do this only once at the end
            # to avoid lots of overhead switching threads.
            self.gui.sliderUpdateTrigger.emit()

    def loop(self):
        hz = self.get_param('rate', 10)  # 10hz

        delta = 0.0

        clock = rclpy.clock.ROSClock()

        # Publish Joint States
        while rclpy.ok():
            start_time = time.time()
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = clock.now().to_msg()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.dependent_joints.items()) > 0
            has_velocity = False
            has_effort = False
            for name, joint in self.free_joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = (len(self.free_joints.items()) +
                          len(self.dependent_joints.items()))
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]

            for i, name in enumerate(self.joint_list):
                msg.name.append(str(name))
                joint = None

                # Add Free Joint
                if name in self.free_joints:
                    joint = self.free_joints[name]
                    factor = 1
                    offset = 0
                # Add Dependent Joint
                elif name in self.dependent_joints:
                    param = self.dependent_joints[name]
                    parent = param['parent']
                    factor = param.get('factor', 1.0)
                    offset = param.get('offset', 0.0)
                    # Handle recursive mimic chain
                    recursive_mimic_chain_joints = [name]
                    while parent in self.dependent_joints:
                        if parent in recursive_mimic_chain_joints:
                            error_message = "Found an infinite recursive mimic chain"
                            self.node.get_logger().error("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                            sys.exit(-1)
                        recursive_mimic_chain_joints.append(parent)
                        param = self.dependent_joints[parent]
                        parent = param['parent']
                        offset += factor * param.get('offset', 0)
                        factor *= param.get('factor', 1)
                    joint = self.free_joints[parent]

                if has_position and 'position' in joint:
                    msg.position[i] = float(joint['position']) * factor + offset
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = float(joint['velocity']) * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = float(joint['effort'])

            if msg.name or msg.position or msg.velocity or msg.effort:
                # Only publish non-empty messages
                self.pub.publish(msg)

            # Spin to make sure we service any subscriptions
            rclpy.spin_once(self.node, timeout_sec=0)

            elapsed = time.time() - start_time

            # TODO(clalancette): Use rclpy.Rate once it is available
            # We want to run at 1.0/hz, but make sure to take into account the
            # amount of time we spent in the loop
            time.sleep((1.0 / hz) - elapsed)

    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward


class JointStatePublisherGui(QWidget):
    sliderUpdateTrigger = Signal()

    def __init__(self, title, jsp, num_rows=0):
        super(JointStatePublisherGui, self).__init__()
        self.jsp = jsp
        self.joint_map = {}
        self.vlayout = QVBoxLayout(self)
        self.scrollable = QWidget()
        self.gridlayout = QGridLayout()
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)

        font = QFont("Helvetica", 9, QFont.Bold)

        ### Generate sliders ###
        sliders = []
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            joint_layout = QVBoxLayout()
            row_layout = QHBoxLayout()

            label = QLabel(name)
            label.setFont(font)
            row_layout.addWidget(label)
            display = QLineEdit("0.00")
            display.setAlignment(Qt.AlignRight)
            display.setFont(font)
            display.setReadOnly(True)
            row_layout.addWidget(display)

            joint_layout.addLayout(row_layout)

            slider = QSlider(Qt.Horizontal)

            slider.setFont(font)
            slider.setRange(0, RANGE)
            slider.setValue(RANGE/2)

            joint_layout.addWidget(slider)

            self.joint_map[name] = {'slidervalue': 0, 'display': display,
                                    'slider': slider, 'joint': joint}
            # Connect to the signal provided by QSignal
            slider.valueChanged.connect(lambda event,name=name: self.onValueChangedOne(name))

            sliders.append(joint_layout)

        # Determine number of rows to be used in grid
        self.num_rows = num_rows
        # if desired num of rows wasn't set, default behaviour is a vertical layout
        if self.num_rows == 0:
            self.num_rows = len(sliders)  # equals VBoxLayout
        # Generate positions in grid and place sliders there
        self.positions = self.generate_grid_positions(len(sliders), self.num_rows)
        for item, pos in zip(sliders, self.positions):
            self.gridlayout.addLayout(item, *pos)

        # Set zero positions read from parameters
        self.center()

        # Synchronize slider and displayed value
        self.sliderUpdate(None)

        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        self.scrollable.setLayout(self.gridlayout)
        self.scroll.setWidget(self.scrollable)
        self.vlayout.addWidget(self.scroll)

        # Buttons for randomizing and centering sliders and
        # Spinbox for on-the-fly selecting number of rows
        self.randbutton = QPushButton('Randomize', self)
        self.randbutton.clicked.connect(self.randomize_event)
        self.vlayout.addWidget(self.randbutton)
        self.ctrbutton = QPushButton('Center', self)
        self.ctrbutton.clicked.connect(self.center_event)
        self.vlayout.addWidget(self.ctrbutton)
        self.maxrowsupdown = QSpinBox()
        self.maxrowsupdown.setMinimum(1)
        self.maxrowsupdown.setMaximum(len(sliders))
        self.maxrowsupdown.setValue(self.num_rows)
        self.maxrowsupdown.valueChanged.connect(self.reorggrid_event)
        self.vlayout.addWidget(self.maxrowsupdown)
        self.setLayout(self.vlayout)

    def onValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        joint_info['slidervalue'] = joint_info['slider'].value()
        joint = joint_info['joint']
        joint['position'] = self.sliderToValue(joint_info['slidervalue'], joint)
        joint_info['display'].setText("%.2f" % joint['position'])

    @pyqtSlot()
    def updateSliders(self):
        self.update_sliders()

    def update_sliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slidervalue'] = self.valueToSlider(joint['position'],
                                                           joint)
            joint_info['slider'].setValue(joint_info['slidervalue'])

    def center_event(self, event):
        self.center()

    def center(self):
        self.jsp.node.get_logger().info("Centering")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(self.valueToSlider(joint['zero'], joint))

    def reorggrid_event(self, event):
        self.reorganize_grid(event)

    def reorganize_grid(self, number_of_rows):
        self.num_rows = number_of_rows

        # Remove items from layout (won't destroy them!)
        items = []
        for pos in self.positions:
            item = self.gridlayout.itemAtPosition(*pos)
            items.append(item)
            self.gridlayout.removeItem(item)

        # Generate new positions for sliders and place them in their new spots
        self.positions = self.generate_grid_positions(len(items), self.num_rows)
        for item, pos in zip(items, self.positions):
            self.gridlayout.addLayout(item, *pos)

    def generate_grid_positions(self, num_items, num_rows):
        if num_rows==0:
          return []
        positions = [(y, x) for x in range(int((math.ceil(float(num_items) / num_rows)))) for y in range(num_rows)]
        positions = positions[:num_items]
        return positions

    def randomize_event(self, event):
        self.randomize()

    def randomize(self):
        self.jsp.node.get_logger().info("Randomizing")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(
                    self.valueToSlider(random.uniform(joint['min'], joint['max']), joint))

    def sliderUpdate(self, event):
        for name, joint_info in self.joint_map.items():
            joint_info['slidervalue'] = joint_info['slider'].value()
        self.update_sliders()

    def valueToSlider(self, value, joint):
        return (value - joint['min']) * float(RANGE) / (joint['max'] - joint['min'])

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue


def main(input_args=None):
    if input_args is None:
        input_args = sys.argv

    # Initialize rclpy with the command-line arguments
    rclpy.init(args=input_args)

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=input_args)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use')
    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])
    with open(parsed_args.urdf_file, 'r') as infp:
        urdf = infp.read()

    node = rclpy.create_node('joint_state_publisher', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    jsp = JointStatePublisher(node, urdf)

    if jsp.gui is None:
        jsp.loop()
    else:
        Thread(target=jsp.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sys.exit(jsp.app.exec_())


if __name__ == '__main__':
    main()
