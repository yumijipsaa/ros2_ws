#!/usr/bin/env python3

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QGroupBox, QLabel, QLineEdit, QPushButton, 
                             QCheckBox, QRadioButton, QTextEdit, QTableWidget,
                             QTableWidgetItem, QButtonGroup, QScrollArea,
                             QSplitter, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor


class RobotControlGUI(QMainWindow):
    """
    PyQt5 GUI for Doosan E0509 Robot Arm Control
    """
    
    def __init__(self, controller_node):
        super().__init__()
        self.controller = controller_node
        self.target_points = []
        
        self.setWindowTitle("Doosan E0509 Robot Arm Controller")
        self.setGeometry(100, 100, 1200, 800)
        
        self.init_ui()
        
        # Update timer for status display
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status_display)
        self.update_timer.start(100)  # Update every 100ms
    
    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)
        
        # Left panel: Control inputs
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # Right panel: Status display
        status_panel = self.create_status_panel()
        splitter.addWidget(status_panel)
        
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(splitter)
    
    def create_control_panel(self):
        """Create control input panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel("Robot Control Panel")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Coordinate mode selection
        coord_group = QGroupBox("Coordinate Mode")
        coord_layout = QVBoxLayout()
        
        self.coord_button_group = QButtonGroup()
        self.absolute_radio = QRadioButton("Absolute Coordinates")
        self.relative_radio = QRadioButton("Relative Coordinates")
        self.absolute_radio.setChecked(True)
        
        self.coord_button_group.addButton(self.absolute_radio)
        self.coord_button_group.addButton(self.relative_radio)
        
        coord_layout.addWidget(self.absolute_radio)
        coord_layout.addWidget(self.relative_radio)
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        # Target position input
        target_group = QGroupBox("Target Position (mm)")
        target_layout = QVBoxLayout()
        
        # X, Y, Z input fields
        xyz_layout = QHBoxLayout()
        self.x_input = QLineEdit()
        self.y_input = QLineEdit()
        self.z_input = QLineEdit()
        
        self.x_input.setPlaceholderText("X")
        self.y_input.setPlaceholderText("Y")
        self.z_input.setPlaceholderText("Z")
        
        xyz_layout.addWidget(QLabel("X:"))
        xyz_layout.addWidget(self.x_input)
        xyz_layout.addWidget(QLabel("Y:"))
        xyz_layout.addWidget(self.y_input)
        xyz_layout.addWidget(QLabel("Z:"))
        xyz_layout.addWidget(self.z_input)
        
        target_layout.addLayout(xyz_layout)
        
        # Add/Clear buttons
        button_layout = QHBoxLayout()
        self.add_point_btn = QPushButton("Add Point")
        self.clear_points_btn = QPushButton("Clear All")
        self.add_point_btn.clicked.connect(self.add_target_point)
        self.clear_points_btn.clicked.connect(self.clear_target_points)
        
        button_layout.addWidget(self.add_point_btn)
        button_layout.addWidget(self.clear_points_btn)
        target_layout.addLayout(button_layout)
        
        # Target points table
        self.points_table = QTableWidget(0, 4)
        self.points_table.setHorizontalHeaderLabels(["#", "X (mm)", "Y (mm)", "Z (mm)"])
        self.points_table.setMaximumHeight(150)
        target_layout.addWidget(self.points_table)
        
        target_group.setLayout(target_layout)
        layout.addWidget(target_group)
        
        # Motion parameters
        params_group = QGroupBox("Motion Parameters")
        params_layout = QVBoxLayout()
        
        # Velocity
        vel_layout = QHBoxLayout()
        self.velocity_check = QCheckBox("Set Velocity (m/s)")
        self.velocity_input = QLineEdit("0.1")
        self.velocity_input.setEnabled(False)
        self.velocity_check.toggled.connect(
            lambda checked: self.velocity_input.setEnabled(checked)
        )
        vel_layout.addWidget(self.velocity_check)
        vel_layout.addWidget(self.velocity_input)
        params_layout.addLayout(vel_layout)
        
        # Acceleration
        acc_layout = QHBoxLayout()
        self.acceleration_check = QCheckBox("Set Acceleration (m/s²)")
        self.acceleration_input = QLineEdit("0.5")
        self.acceleration_input.setEnabled(False)
        self.acceleration_check.toggled.connect(
            lambda checked: self.acceleration_input.setEnabled(checked)
        )
        acc_layout.addWidget(self.acceleration_check)
        acc_layout.addWidget(self.acceleration_input)
        params_layout.addLayout(acc_layout)
        
        params_group.setLayout(params_layout)
        layout.addWidget(params_group)
        
        # Execute button
        self.execute_btn = QPushButton("Execute Motion")
        self.execute_btn.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; "
            "font-size: 16px; padding: 10px; font-weight: bold; }"
            "QPushButton:hover { background-color: #45a049; }"
        )
        self.execute_btn.clicked.connect(self.execute_motion)
        layout.addWidget(self.execute_btn)
        
        # Stop button
        self.stop_btn = QPushButton("Emergency Stop")
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; "
            "font-size: 14px; padding: 8px; font-weight: bold; }"
            "QPushButton:hover { background-color: #da190b; }"
        )
        self.stop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.stop_btn)
        
        layout.addStretch()
        
        return panel
    
    def create_status_panel(self):
        """Create status display panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel("Robot Status Monitor")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)
        
        # Connection status
        status_group = QGroupBox("Connection Status")
        status_layout = QVBoxLayout()
        
        self.connection_label = QLabel("Disconnected")
        self.connection_label.setStyleSheet(
            "background-color: #f44336; color: white; "
            "padding: 5px; font-weight: bold;"
        )
        status_layout.addWidget(self.connection_label)
        
        self.motion_label = QLabel("Status: Idle")
        status_layout.addWidget(self.motion_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Joint angles
        joint_group = QGroupBox("Joint Angles (degrees)")
        joint_layout = QVBoxLayout()
        
        self.joint_labels = []
        for i in range(6):
            label = QLabel(f"Joint {i+1}: 0.00°")
            self.joint_labels.append(label)
            joint_layout.addWidget(label)
        
        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)
        
        # Current position
        pos_group = QGroupBox("Current Position (Base Frame)")
        pos_layout = QVBoxLayout()
        
        self.pos_x_label = QLabel("X: 0.00 mm")
        self.pos_y_label = QLabel("Y: 0.00 mm")
        self.pos_z_label = QLabel("Z: 0.00 mm")
        
        pos_layout.addWidget(self.pos_x_label)
        pos_layout.addWidget(self.pos_y_label)
        pos_layout.addWidget(self.pos_z_label)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # Log display
        log_group = QGroupBox("Real-time Log")
        log_layout = QVBoxLayout()
        
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setMaximumHeight(200)
        log_layout.addWidget(self.log_display)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        layout.addStretch()
        
        return panel
    
    def add_target_point(self):
        """Add target point to list"""
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
            
            self.target_points.append((x, y, z))
            
            # Add to table
            row = self.points_table.rowCount()
            self.points_table.insertRow(row)
            self.points_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.points_table.setItem(row, 1, QTableWidgetItem(f"{x:.2f}"))
            self.points_table.setItem(row, 2, QTableWidgetItem(f"{y:.2f}"))
            self.points_table.setItem(row, 3, QTableWidgetItem(f"{z:.2f}"))
            
            # Clear inputs
            self.x_input.clear()
            self.y_input.clear()
            self.z_input.clear()
            
            self.controller.add_log(f"Added target point: ({x}, {y}, {z})")
            
        except ValueError:
            self.controller.add_log("ERROR: Invalid coordinate values")
    
    def clear_target_points(self):
        """Clear all target points"""
        self.target_points.clear()
        self.points_table.setRowCount(0)
        self.controller.add_log("Cleared all target points")
    
    def execute_motion(self):
        """Execute motion to target points"""
        if not self.target_points:
            self.controller.add_log("ERROR: No target points defined")
            return
        
        # Get coordinate mode
        coord_mode = "absolute" if self.absolute_radio.isChecked() else "relative"
        
        # Get velocity and acceleration
        velocity = float(self.velocity_input.text()) if self.velocity_check.isChecked() else 0.1
        acceleration = float(self.acceleration_input.text()) if self.acceleration_check.isChecked() else 0.5
        
        # Execute motion
        success = self.controller.execute_motion(
            self.target_points,
            coord_mode,
            velocity,
            acceleration
        )
        
        if success:
            self.execute_btn.setEnabled(False)
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        self.controller.stop_motion()
        self.execute_btn.setEnabled(True)
    
    def update_status_display(self):
        """Update status display from robot state"""
        state = self.controller.get_robot_state()
        
        # Connection status
        if state['connected']:
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet(
                "background-color: #4CAF50; color: white; "
                "padding: 5px; font-weight: bold;"
            )
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet(
                "background-color: #f44336; color: white; "
                "padding: 5px; font-weight: bold;"
            )
        
        # Motion status
        if state['moving']:
            self.motion_label.setText("Status: Moving")
            self.execute_btn.setEnabled(False)
        else:
            self.motion_label.setText("Status: Idle")
            self.execute_btn.setEnabled(True)
        
        # Joint angles
        import math
        for i, angle in enumerate(state['joint_angles']):
            deg = math.degrees(angle)
            self.joint_labels[i].setText(f"Joint {i+1}: {deg:.2f}°")
        
        # Position
        pos = state['position']
        self.pos_x_label.setText(f"X: {pos['x']:.2f} mm")
        self.pos_y_label.setText(f"Y: {pos['y']:.2f} mm")
        self.pos_z_label.setText(f"Z: {pos['z']:.2f} mm")
        
        # Logs
        log_text = "\n".join(state['logs'][-20:])  # Show last 20 logs
        if log_text != self.log_display.toPlainText():
            self.log_display.setPlainText(log_text)
            self.log_display.verticalScrollBar().setValue(
                self.log_display.verticalScrollBar().maximum()
            )
