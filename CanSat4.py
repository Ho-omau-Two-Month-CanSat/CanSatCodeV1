import sys
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QPushButton, QMainWindow, QLabel, QLineEdit, QSizePolicy, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit
from PyQt6.QtCore import QTimer, Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import csv
import math
import serial
import io

t = []
height = []
roll = []
pitch = []
yaw = []
x = []
y = []
z = []
g = []


class MyMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setGeometry(0,0,1920,780)

        ## Layout
        # Create the main widget and set it as central
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)

        # Create layout for the main widget
        main_layout = QVBoxLayout(main_widget)

        font = self.font()
        font.setPointSize(14)
        font.setBold = True

        # Label for title
        title_widget = QWidget(main_widget)
        main_layout.addWidget(title_widget)
        top_layout = QHBoxLayout(title_widget)

        self.title = QLabel("Cansat")
        self.title.setFont(font)
        top_layout.addWidget(self.title)

        self.blank =QLabel("")
        top_layout.addWidget(self.blank)

        self.status = QLabel("Status: ")
        self.status.setFont(font)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_layout.addWidget(self.status)

        # Create a widget for labels
        label_widget = QWidget(main_widget)
        main_layout.addWidget(label_widget)

        # Create a horizontal layout for the labels
        label_layout = QHBoxLayout(label_widget)

        # Label for velocity
        v = QLabel("Velocity (m/s):")
        v.setFont(font)
        label_layout.addWidget(v)

        # Create a label to display the velocity
        self.velocity_label = QLabel("0.0")
        self.velocity_label.setFont(font)
        label_layout.addWidget(self.velocity_label)

        # Label for max speed
        maxS = QLabel("Max Speed (m/s):")
        maxS.setFont(font)
        label_layout.addWidget(maxS)

        # Create a label to display the max speed
        self.maxS_label = QLabel("0.0")
        self.maxS_label.setFont(font)
        label_layout.addWidget(self.maxS_label)

        # Label for acceleration
        a = QLabel("Acceleration (g)):")
        a.setFont(font)
        label_layout.addWidget(a)

        # Create a label to display the acceleration
        self.acceleration_label = QLabel("0.0")
        self.acceleration_label.setFont(font)
        label_layout.addWidget(self.acceleration_label)

        # Label for max acceleration
        maxA = QLabel("Max Acceleration (g):")
        maxA.setFont(font)
        label_layout.addWidget(maxA)

        # Create a label to display the max acceleration
        self.maxA_label = QLabel("0.0")
        self.maxA_label.setFont(font)
        label_layout.addWidget(self.maxA_label)        

        # Label for temp
        temp = QLabel("Temperature (C):")
        temp.setFont(font)
        label_layout.addWidget(temp)

        # Create a label to display the temperature
        self.temp_label = QLabel("0.0")
        self.temp_label.setFont(font)
        label_layout.addWidget(self.temp_label)               

        # Create a widget for graphs
        graph_widget = QWidget(main_widget)
        main_layout.addWidget(graph_widget)

        # Create a layout for the graphs
        graph_layout = QHBoxLayout(graph_widget)

        # Create Matplotlib figure and canvas for the graphs
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        graph_layout.addWidget(self.canvas)

        # Add graphs 
        self.plot_graphs()

        # Label for data section
        d = QLabel("Data")
        d.setFont(font)
        main_layout.addWidget(d)

        # Create a widget for data display
        self.initSerial()
        self.data_widget = QTextEdit(main_widget)
        self.data_widget.setReadOnly(True)
        self.data_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        main_layout.addWidget(self.data_widget)

        self.start_button = QPushButton('Start Receiving')
        self.start_button.clicked.connect(self.startReceiving)
        main_layout.addWidget(self.start_button)

        #Button widget sending packets
        button_widget = QWidget(main_widget)
        main_layout.addWidget(button_widget)
        button_layout = QHBoxLayout(button_widget)

        self.send_Z = QPushButton("Zero")
        button_layout.addWidget(self.send_Z)
        self.send_Z.clicked.connect(lambda: self.sendPacket("z"))
        
        self.send_R = QPushButton("Release")
        button_layout.addWidget(self.send_R)
        self.send_R.clicked.connect(lambda:self.sendPacket("r"))

        self.send_L = QPushButton("Launch Ready")
        button_layout.addWidget(self.send_L)
        self.send_L.clicked.connect(lambda:self.sendPacket("l"))

        self.send_P = QPushButton("Parachute")
        button_layout.addWidget(self.send_P)
        self.send_P.clicked.connect(lambda:self.sendPacket("p"))

        self.send_A = QPushButton("Attach Servos")
        button_layout.addWidget(self.send_A)
        self.send_A.clicked.connect(lambda:self.sendPacket("a"))

    ###
    # Xbee receive
    def initSerial(self):
        self.serial_port = serial.Serial('COM6', baudrate=9600, timeout=1)

    def startReceiving(self):
        self.start_button.setEnabled(False)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.receive_get_data)
        self.timer.start(100)  # Check for incoming data every 1 second

    def receive_get_data(self):
        if self.serial_port.in_waiting > 0:
            received_data = self.serial_port.readline().decode('utf-8').strip()
            if (received_data):
                self.data_widget.append(f'Received: {received_data}')
                csv_file = io.StringIO(received_data)
                reader = csv.reader(csv_file)
                for line in reader: 
                    time = self.convert_time(str(line[1]))
                    t.append(time)
                    height.append(float(line[5]))
                    roll.append(float(line[8]))
                    pitch.append(float(line[9]))
                    yaw.append(float(line[10]))
                    x.append(float(line[11]))
                    y.append(float(line[12]))
                    z.append(float(line[13]))
                    g.append(float(line[17]))
                    
                    if len(t) == 30:
                        del t[0]
                        del height[0]
                        del roll[0]
                        del pitch[0]
                        del yaw[0]
                        del x[0]
                        del y[0]
                        del z[0]
                        del g[0]
                    
                    battery = (float(line[7])/6)*100
                    self.status.setText(f"Status: {line[3]}, Release: {line[4]}, Parachute: {line[19]}")
                    self.velocity_label.setText(f"{line[15]}")
                    self.temp_label.setText(f"{line[6]}")
                    self.maxS_label.setText(f"{line[16]}")
                    self.acceleration_label.setText(f"{line[17]}")
                    self.maxA_label.setText(f"{line[18]}")
                    self.title.setText(f"CanSat: {battery: .2f} percent battery")
                
                self.update_data_and_plots()

            
    #xbee send
    def sendPacket(self, send):
        self.packet_data = send
        self.serial_port.write(self.packet_data.encode('utf-8'))

    def store_input(self):
        user_input = self.input_text.text()
        self.stored_input = user_input

    def closeEvent(self, event):
        self.serial_port.close()
        super().closeEvent(event)
    ### 

    # initial graph set up
    def plot_graphs(self):
        # Plot data on the Matplotlib figure
        self.figure.clf()
        #Temperature
        ax = self.figure.add_subplot(131)
        ax.plot(t, x, 'r', label='X')
        ax.plot(t, y, 'g', label='Y')
        ax.plot(t, z, 'b', label='Z')
        ax.plot(t, g, label ='g')
        ax.legend(loc="upper left")
        ax.set_title("Acceleration")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (g)")

        #Height 
        ax = self.figure.add_subplot(132)
        ax.plot(t, height)
        ax.set_title("Height")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Height (meters)")

        #Rotation
        ax = self.figure.add_subplot(133)
        ax.plot(t, roll, 'r', label='Roll')
        ax.plot(t, pitch, 'g', label='Pitch')
        ax.plot(t, yaw, 'b', label='Yaw')
        ax.legend(loc="upper left")
        ax.set_title("Rotation")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Rotation (degrees/s)")

        self.figure.tight_layout()

        # Refresh the canvas
        self.canvas.draw()

    def update_data_and_plots(self):
        # Update data and graph 
        self.plot_graphs()

    def convert_time(self,time_str):
        # Extract hours
        hours = int(time_str[0:2])
    
        # Extract minutes
        minutes = int(time_str[3:5])
    
        # Extract seconds
        seconds = float(time_str[6:])
    
        # Calculate total seconds
        total_seconds = hours * 3600 + minutes * 60 + seconds
    
        return total_seconds

#Main execution, leave alone 
def main():
    app = QApplication(sys.argv)
    main_window = MyMainWindow()
    main_window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
