# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys
# subprocess
import subprocess
import time
import threading

# Changing this to QEApplication to see if it will updatae...
from PySide2.QtWidgets import QApplication, QWidget, QPushButton, QLineEdit, QLabel
from PySide2.QtCore import QFile, QTimer
from PySide2.QtUiTools import QUiLoader

# Defining our threading function
"""
def move_position():
    print("Starting background task...")
    desired_pos_number = float(self.desired_pos.text())    
    initial_pos = subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8")
    initial_pos = float(initial_pos.split()[1])

    # initial position below? move up!
    if initial_pos < desired_pos_number:
        subprocess.run(["caput", "testETH32:motors:M1For", "1"])
        while initial_pos < desired_pos_number:
            initial_pos = float(subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8").split()[1])
            time.sleep(0.1)
    # initial position above? move down! 
    if initial_pos > desired_pos_number: 
        subprocess.run(["caput", "testETH32:motors:M1Back", "1"])
        while initial_pos > desired_pos_number:
            initial_pos = float(subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8").split()[1])
            time.sleep(0.1)

    if initial_pos == desired_pos_number:
        subprocess.run(["caput", "testETH32:motors:M1For", "0"])
        subprocess.run(["caput", "testETH32:motors:M1Back", "0"])
        subprocess.run(["caput", "testETH32:motors:Run", "0"])    

    print("Background task finished.")
"""

class Widget(QWidget):

    def __init__(self):
        super(Widget, self).__init__()
        self.load_ui()

        # finding and naming elements in the ui
        for child in self.findChildren(QPushButton):
            print(f"Found button: {child.objectName()}")
        for child in self.findChildren(QLineEdit):
            print(f"Found line edit: {child.objectName()}")
        for child in self.findChildren(QLabel):
            print(f"Found line edit: {child.objectName()}")

        # calling functions and defining variables for specific ui elements
        self.automatic_pos = self.findChild(QPushButton, "automatic_control")
        self.desired_pos = self.findChild(QLineEdit, "desired_pos")
        self.error_status = self.findChild(QLabel, "error_status")
        print(f"Button selected {self.automatic_pos}")
        print(f"User input selected {self.desired_pos}")
        print(f"Text box output selected {self.error_status}")
        if self.automatic_pos:
            self.automatic_pos.clicked.connect(self.update_position)
        self.error_status.setText("The most recent auomatic control message \n will be placed here!")

    def load_ui(self):
        loader = QUiLoader()
        path = os.fspath(Path(__file__).resolve().parent / "motorcontrol.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        loader.load(ui_file, self)
        ui_file.close()    
    
    def move_position(self):
        print("Starting background task...")
        desired_pos_number = float(self.desired_pos.text())    
        initial_pos = subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8")
        initial_pos = float(initial_pos.split()[1])

    # initial position below? move up!
        if initial_pos < desired_pos_number:
            subprocess.run(["caput", "testETH32:motors:M1For", "1"])
            while initial_pos < desired_pos_number:
                initial_pos = float(subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8").split()[1])
                time.sleep(0.1)
    # initial position above? move down! 
        if initial_pos > desired_pos_number: 
            subprocess.run(["caput", "testETH32:motors:M1Back", "1"])
            while initial_pos > desired_pos_number:
                initial_pos = float(subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8").split()[1])
                time.sleep(0.1)

        if initial_pos == desired_pos_number:
            subprocess.run(["caput", "testETH32:motors:M1For", "0"])
            subprocess.run(["caput", "testETH32:motors:M1Back", "0"])
            subprocess.run(["caput", "testETH32:motors:Run", "0"])    

        print("Background task finished.")

    def update_position(self):
        print(f"We're going to update the positon to {self.desired_pos.text()}!")
        try:
            float(self.desired_pos.text())
            desired_pos_number = float(self.desired_pos.text())
            print(f"{self.desired_pos.text()} is a valid number!")
            self.error_status.setText(f"We're going to update the \n positon to {self.desired_pos.text()}!")
            # print(f"{round(desired_pos_number/0.05, 3)}")
            # print(f"{int(desired_pos_number/0.05)}")
            # selecting only valid inputs  previous check: float.is_integer(desired_pos_number/0.05)
            if 0 <= desired_pos_number <= 10: # and int(desired_pos_number)/0.05 == True:
                subprocess.run(["caput", "testETH32:motors:Run", "1"])
                initial_pos = subprocess.check_output(["caget", "testETH32:motors:M1Pos_RBV"]).decode("utf-8")
                initial_pos = float(initial_pos.split()[1])
                print(f"initial positon is {initial_pos}!")
                thread = threading.Thread(target=self.move_position)
                thread.start()

            else:
                print("Valid inptus are between 0 and 10 and divisble by 0.05.")
                self.error_status.setText(f"{self.desired_pos.text()} is not a valid input! \n Valid inptus are between 0 and 10 \n and divisble by 0.05.")

        except ValueError:  
            print("You did not input a number :3")
            self.error_status.setText(f"{self.desired_pos.text()} is not a valid input! \n Valid inptus are between 0 and 10 \n and divisble by 0.05.")
        

if __name__ == "__main__":
    app = QApplication([])
    widget = Widget()
    widget.show()
    sys.exit(app.exec_())

