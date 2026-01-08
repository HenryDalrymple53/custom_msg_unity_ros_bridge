import sys
from PyQt6.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, 
                             QHBoxLayout, QScrollArea, QCheckBox, QFrame, QPushButton, QFileDialog)
from PyQt6.QtCore import Qt
from processes import Processes
import os

class JsonGenGUI:
    def __init__(self):
        # Initialize ROS 2 message processor
        self.proc = Processes()
        self.proc.process_msgs()
        self.proc.assign_libs_and_count()

        self.dir_path = os.getcwd()
        print(self.dir_path)


        # Initialize Qt Application
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("ROS2 Message Overview")
        self.window.setGeometry(100, 100, 1200, 800)

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(10)

        # Top section (1/3 of window) - Header labels
        top_section = QWidget()
        top_layout = QVBoxLayout()
        top_layout.setSpacing(15)
        
        self.main_label = QLabel("ROS2 Message Overview")
        self.main_label.setStyleSheet("font-size: 24px; font-weight: bold;")
        
        self.button = QPushButton("Process messages")
        self.button.clicked.connect(self.on_button_clicked)

        self.dir_button = QPushButton("Select Directory to save JSON...")
        self.dir_button.clicked.connect(self.select_directory)
        
        top_layout.addWidget(self.main_label)
        top_layout.addWidget(self.button)
        top_layout.addWidget(self.dir_button)
        

        top_layout.addStretch()
        
        top_section.setLayout(top_layout)

        # Bottom section (2/3 of window) - Three scroll areas
        bottom_section = QWidget()
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(10)
        bottom_layout.setContentsMargins(0, 0, 0, 0)

        # Create three scroll areas
        self.msg_scroll = self.create_scroll_area("Messages", self.proc.message_libs)
        self.action_scroll = self.create_scroll_area("Actions", self.proc.action_libs)
        self.service_scroll = self.create_scroll_area("Services", self.proc.service_libs)

        bottom_layout.addWidget(self.msg_scroll)
        bottom_layout.addWidget(self.action_scroll)
        bottom_layout.addWidget(self.service_scroll)
        
        bottom_section.setLayout(bottom_layout)

        # Add sections to main layout with stretch factors
        main_layout.addWidget(top_section, 1)  # 1/3
        main_layout.addWidget(bottom_section, 2)  # 2/3

        self.window.setLayout(main_layout)


        # Show window
        self.window.show()
        sys.exit(self.app.exec())

    def create_scroll_area(self, title, data_dict):
        """Create a scroll area with checkboxes for each library"""
        # Main frame
        frame = QFrame()
        frame.setFrameShape(QFrame.Shape.StyledPanel)
        frame.setStyleSheet("QFrame { border: 2px solid #ccc; border-radius: 5px; }")
        
        frame_layout = QVBoxLayout()
        frame_layout.setContentsMargins(5, 5, 5, 5)
        frame_layout.setSpacing(5)
        
        # Title label
        title_label = QLabel(title)
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; padding: 5px;")
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        frame_layout.addWidget(title_label)
        
        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        
        # Content widget inside scroll area
        content_widget = QWidget()
        content_layout = QVBoxLayout()
        content_layout.setSpacing(8)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        # Add checkbox for each library with count
        for lib_name, messages in sorted(data_dict.items()):
            checkbox = QCheckBox(f"{lib_name}")
            checkbox.setStyleSheet("QCheckBox { padding: 5px; }")
            checkbox.stateChanged.connect(lambda state, lib=lib_name, t=title: 
                                         self.on_checkbox_changed(state, lib, t))
            content_layout.addWidget(checkbox)
        
        content_widget.setLayout(content_layout)
        scroll.setWidget(content_widget)
        
        frame_layout.addWidget(scroll)
        frame.setLayout(frame_layout)
        
        return frame


    def on_checkbox_changed(self, state, lib_name, category):
        """Handle checkbox state changes"""
        is_checked = state == Qt.CheckState.Checked.value
        print(f"{category} - {lib_name}: {'Checked' if is_checked else 'Unchecked'}")
        if lib_name in self.proc.message_libs:
            self.proc.message_libs[lib_name] = is_checked
        if lib_name in self.proc.service_libs:
            self.proc.service_libs[lib_name] = is_checked
        if lib_name in self.proc.action_libs:
            self.proc.action_libs[lib_name] = is_checked


    def on_button_clicked(self):
        """This function (slot) runs when the button is clicked."""
        print("Processing...")
        self.proc.convert_to_json(self.dir_path)

    def select_directory(self):
        print("Here")
        self.dir_path = QFileDialog.getExistingDirectory(
            caption="Select directory",
            directory = self.dir_path,
            options=QFileDialog.Option.DontUseNativeDialog,
        )
        


if __name__ == "__main__":
    gui = JsonGenGUI()