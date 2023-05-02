import PyQt5
import sys

import rclpy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from rclpy.node import Node

from PyQt5.QtCore import Qt
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDateEdit,
    QDateTimeEdit,
    QFileDialog,
    QDial,
    QGridLayout,
    QDoubleSpinBox,
    QFontComboBox,
    QLabel,
    QLCDNumber,
    QLineEdit,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QHBoxLayout,
    QRadioButton,
    QSlider,
    QSpinBox,
    QTimeEdit,
    QVBoxLayout,
    QWidget,
    QScrollArea,
    QSizePolicy,
)
from PyQt5.QtCore import QRect
from PyQt5.QtGui import QPixmap

pkg_path = os.path.join(get_package_share_directory('super_3d_cart'))
assets = os.path.join(pkg_path,'launch')

wheels_d = '0.033'
wheels_width = '0.026'
c_length = '0.335'
c_width = '0.265'
c_height = '0.138'
world_file = 'worlds/empty.world'
rear_camera = 'False'
front_camera = 'False'
fl_sensor = '-'
fr_sensor = '-'
sl_sensor = '-'
sr_sensor = '-'
rl_sensor = '-'
rr_sensor = '-'


class WheelOptions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Настройки колес")
        self.resize(550, 300)
        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName(u"centralwidget")
        self.widget = QWidget(self.centralwidget)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(50, 30, 456, 140))
        self.verticalLayout_7 = QVBoxLayout(self.widget)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label = QLabel(self.widget, text='Размер')
        self.label.setObjectName(u"label")

        self.verticalLayout_5.addWidget(self.label)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_3 = QLabel(self.widget, text='Радиус')
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout.addWidget(self.label_3)

        self.label_4 = QLabel(self.widget, text='Ширина')
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout.addWidget(self.label_4)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.lineEdit = QLineEdit(self.widget, text='0.033')
        self.lineEdit.setObjectName(u"lineEdit")

        self.verticalLayout_2.addWidget(self.lineEdit)

        self.lineEdit_3 = QLineEdit(self.widget, text='0.026')
        self.lineEdit_3.setObjectName(u"lineEdit_3")

        self.verticalLayout_2.addWidget(self.lineEdit_3)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_5.addLayout(self.horizontalLayout)

        self.horizontalLayout_3.addLayout(self.verticalLayout_5)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.label_2 = QLabel(self.widget, text='Положение относительно робота')
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout_6.addWidget(self.label_2)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_5 = QLabel(self.widget, text='X  ')
        self.label_5.setObjectName(u"label_5")
        self.horizontalLayout_3.setSpacing(20)
        self.verticalLayout_3.addWidget(self.label_5)

        self.label_6 = QLabel(self.widget, text='Y  ')
        self.label_6.setObjectName(u"label_6")

        self.verticalLayout_3.addWidget(self.label_6)

        self.horizontalLayout_2.addLayout(self.verticalLayout_3)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.lineEdit_2 = QLineEdit(self.widget)
        self.lineEdit_2.setObjectName(u"lineEdit_2")
        self.verticalLayout_4.addWidget(self.lineEdit_2)
        self.lineEdit_4 = QLineEdit(self.widget)
        self.lineEdit_4.setObjectName(u"lineEdit_4")
        self.verticalLayout_4.addWidget(self.lineEdit_4)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.verticalLayout_6.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3.addLayout(self.verticalLayout_6)
        self.verticalLayout_7.addLayout(self.horizontalLayout_3)

        self.pushButton = QPushButton(self.widget, text='Применить')
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.clicked.connect(self.apply)

        self.verticalLayout_7.addWidget(self.pushButton)

        self.setCentralWidget(self.centralwidget)

    def apply(self):
        global wheels_d, wheels_width
        wheels_d = self.lineEdit.text()
        wheels_width =  self.lineEdit_3.text()
        self.close()

class PositionOptions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Начальное положение")
        self.widget = QWidget(self)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(30, 20, 271, 231))
        self.verticalLayout = QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(self.widget, text="X  ")
        self.label.setObjectName(u"label")
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QLineEdit(self.widget)
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QHBoxLayout()
        self.label_3 = QLabel(self.widget, text="Y  ")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.lineEdit_3 = QLineEdit(self.widget)
        self.horizontalLayout_3.addWidget(self.lineEdit_3)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2 = QHBoxLayout()
        self.label_2 = QLabel(self.widget, text="Z  ")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.lineEdit_2 = QLineEdit(self.widget)
        self.horizontalLayout_2.addWidget(self.lineEdit_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_4 = QHBoxLayout()
        self.label_4 = QLabel(self.widget, text="Угол")
        self.horizontalLayout_4.addWidget(self.label_4)
        self.lineEdit_4 = QLineEdit(self.widget)
        self.horizontalLayout_4.addWidget(self.lineEdit_4)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.pushButton = QPushButton(self.widget, text="Применить")
        self.pushButton.clicked.connect(self.close)
        self.verticalLayout.addWidget(self.pushButton)
        self.setCentralWidget(self.widget)

class SizeOptions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Геометрические параметры")
        self.widget = QWidget(self)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(30, 20, 271, 231))
        self.verticalLayout = QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label = QLabel(self.widget, text="Длина")
        self.label.setObjectName(u"label")
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit = QLineEdit(self.widget, text="0.335")
        self.horizontalLayout.addWidget(self.lineEdit)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3 = QHBoxLayout()
        self.label_3 = QLabel(self.widget, text="Ширина")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.lineEdit_3 = QLineEdit(self.widget, text="0.265")
        self.horizontalLayout_3.addWidget(self.lineEdit_3)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2 = QHBoxLayout()
        self.label_2 = QLabel(self.widget, text="Высота")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.lineEdit_2 = QLineEdit(self.widget, text="0.138")
        self.horizontalLayout_2.addWidget(self.lineEdit_2)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.pushButton = QPushButton(self.widget, text="Применить")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton.clicked.connect(self.apply)
        self.setCentralWidget(self.widget)

    def apply(self):
        global c_length, c_width, c_height
        c_length = self.lineEdit.text()
        c_width = self.lineEdit_3.text()
        c_height = self.lineEdit_2.text()
        self.close()



class RobotOptions(QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(800, 750)
        self.setWindowTitle("Настройки робота")
        self.label = QLabel(self)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 10, 421, 581))
        self.label.setPixmap(QPixmap(os.path.join(assets, "options.png")))
        self.pushButton_4 = QPushButton(self, text='Применить')
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(20, 610, 761, 71))
        self.pushButton_4.clicked.connect(self.complete)
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_4.sizePolicy().hasHeightForWidth())
        self.pushButton_4.setSizePolicy(sizePolicy)
        self.comboBox_lf = QComboBox(self)
        self.comboBox_lf.setGeometry(QRect(99, 21, 102, 30))
        self.comboBox_lf.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_rf = QComboBox(self)
        self.comboBox_rf.setGeometry(QRect(232, 21, 102, 30))
        self.comboBox_rf.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_lr = QComboBox(self)
        self.comboBox_lr.setGeometry(QRect(99, 471, 102, 30))
        self.comboBox_lr.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_rr = QComboBox(self)
        self.comboBox_rr.setGeometry(QRect(232, 471, 102, 30))
        self.comboBox_rr.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_lc = QComboBox(self)
        self.comboBox_lc.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_lc.setGeometry(QRect(23, 247, 102, 30))
        self.comboBox_rc = QComboBox(self)
        self.comboBox_rc.addItems(['-', 'УЗ датчик', 'ИК датчик', 'Лидар'])
        self.comboBox_rc.setGeometry(QRect(305, 247, 102, 30))
        self.comboBox_uc = QComboBox(self)
        self.comboBox_uc.setGeometry(QRect(164, 148, 102, 30))
        self.comboBox_dc = QComboBox(self)
        self.comboBox_dc.setGeometry(QRect(164, 344, 102, 30))
        self.comboBox_uc.addItems(['-', 'Камера'])
        self.comboBox_dc.addItems(['-', 'Камера'])
        self.widget = QWidget(self)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(480, 60, 281, 481))
        self.verticalLayout = QVBoxLayout(self.widget)
        self.verticalLayout.setSpacing(150)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.pushButton = QPushButton(self.widget, text='Геометрические параметры')
        self.pushButton.clicked.connect(self.show_size_options)
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy1)

        self.verticalLayout.addWidget(self.pushButton)

        self.pushButton_2 = QPushButton(self.widget, text='Настройки колес')
        self.pushButton_2.setObjectName(u"pushButton_2")
        sizePolicy1.setHeightForWidth(self.pushButton_2.sizePolicy().hasHeightForWidth())
        self.pushButton_2.setSizePolicy(sizePolicy1)
        self.pushButton_2.clicked.connect(lambda: wheel_options.show())
        self.verticalLayout.addWidget(self.pushButton_2)

        self.pushButton_3 = QPushButton(self.widget, text='Начальное положение')
        self.pushButton_3.setObjectName(u"pushButton_3")
        sizePolicy1.setHeightForWidth(self.pushButton_3.sizePolicy().hasHeightForWidth())
        self.pushButton_3.setSizePolicy(sizePolicy1)
        self.pushButton_3.clicked.connect(self.show_pos_options)
        self.verticalLayout.addWidget(self.pushButton_3)


    def show_size_options(self):
        global size_options
        size_options.show()

    def show_pos_options(self):
        global pos_options
        pos_options.show()
    
    def complete(self):
        global front_camera, rear_camera
        global fl_sensor, fr_sensor, sl_sensor, sr_sensor, rl_sensor, rr_sensor
        self.on_complete()
        if self.comboBox_uc.currentText() == 'Камера':
            front_camera = 'True'
        else:
            front_camera = 'False'
        if self.comboBox_dc.currentText() == 'Камера':
            rear_camera = 'True'
        else:
            rear_camera = 'False'
        dict = {}
        dict['УЗ датчик'] = 'us_sensor'
        dict['ИК датчик'] = 'ir_sensor'
        dict['Лидар'] = 'lidar_sensor'
        dict['-'] = '-'
        fl_sensor = dict[self.comboBox_lf.currentText()]
        fr_sensor = dict[self.comboBox_rf.currentText()]
        sl_sensor = dict[self.comboBox_lc.currentText()]
        sr_sensor = dict[self.comboBox_rc.currentText()]
        rl_sensor = dict[self.comboBox_lr.currentText()]
        rr_sensor = dict[self.comboBox_rr.currentText()]
        self.close()

    def setOnCompleted(self, func):
        self.on_complete = func


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Super 3D Cart")
        self.resize(200, 180)
        self.centralwidget = QWidget(self)
        self.widget = QWidget(self.centralwidget)
        self.widget.setGeometry(QRect(30, 20, 241, 231))
        self.verticalLayout_3 = QVBoxLayout(self.widget)
        self.verticalLayout_3.setContentsMargins(10, 10, 10, 10)
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.pushButton = QPushButton(self.widget, text='Настройки робота')
        self.pushButton.clicked.connect(self.show_robot_options)
        self.verticalLayout.addWidget(self.pushButton)

        self.pushButton_2 = QPushButton(self.widget, text='Загрузить файл пространства')
        self.pushButton_2.clicked.connect(self.show_urdf_picker)
        self.verticalLayout.addWidget(self.pushButton_2)

        self.pushButton_3 = QPushButton(self.widget, text='Загрузить файл команд')
        self.pushButton_3.clicked.connect(self.show_py_picker)
        self.verticalLayout.addWidget(self.pushButton_3)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label = QLabel(self.widget)
        self.label.setObjectName(u"label")
        self.label.setPixmap(QPixmap(os.path.join(assets, "led_r.png")))

        self.verticalLayout_2.addWidget(self.label)

        self.label_2 = QLabel(self.widget)
        self.label_2.setPixmap(QPixmap(os.path.join(assets, "led_r.png")))

        self.verticalLayout_2.addWidget(self.label_2)

        self.label_3 = QLabel(self.widget)
        self.label_3.setPixmap(QPixmap(os.path.join(assets, "led_r.png")))

        self.verticalLayout_2.addWidget(self.label_3)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_3.addLayout(self.horizontalLayout)

        self.pushButton_4 = QPushButton(self.widget, text='Запуск')
        self.pushButton_4.clicked.connect(self.close)

        self.verticalLayout_3.addWidget(self.pushButton_4)

        self.setCentralWidget(self.widget)


    def show_urdf_picker(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self, "Выберите файл пространства", "","All Files (*);;World files (*.world);;Urdf files (*.urdf)", options=options)
        if fileName:
            global world_file
            world_file = fileName
            self.label_2.setPixmap(QPixmap(os.path.join(assets, "led.png")))

    def show_py_picker(self):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self, "Выберите файл команд", "","Python files (*.py);; All Files (*)", options=options)
        if fileName:
            self.label_3.setPixmap(QPixmap(os.path.join(assets, "led.png")))

    def show_robot_options(self):
        options.setOnCompleted(self.turn_led_green)
        options.show()
        return
    
    def turn_led_green(self):
        self.label.setPixmap(QPixmap(os.path.join(assets, "led.png")))


def run_app():
    global options
    global size_options
    global pos_options
    global wheel_options
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    options = RobotOptions()
    size_options = SizeOptions()
    pos_options = PositionOptions()
    wheel_options = WheelOptions()
    app.exec()

import os

def generate_launch_description():
    package_name='super_3d_cart'
    global wheels_d, wheels_width, c_length, c_width, c_height, world_file, front_camera, rear_camera, fl_sensor, fr_sensor, sl_sensor, sr_sensor, rl_sensor, rr_sensor
    run_app()
    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_sim.launch.py'
                )]), launch_arguments={
                        'wheels_d': wheels_d,
                        'wheels_width': wheels_width,
                        'c_length': c_length,
                        'c_width': c_width,
                        'c_height': c_height, 
                        'world_file': world_file,
                        'front_camera': front_camera,
                        'rear_camera': rear_camera,
                        'fl_sensor': fl_sensor,
                        'fr_sensor': fr_sensor,
                        'sl_sensor': sl_sensor,
                        'sr_sensor': sr_sensor,
                        'rl_sensor': rl_sensor,
                        'rr_sensor': rr_sensor,
                    }.items()
    )
    return LaunchDescription([
        sim
    ])