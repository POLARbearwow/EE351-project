import sys
import smbus
from time import sleep
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import numpy as np

# MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

class kalman_filter:
    def __init__(self,Q,R):
        self.Q = Q
        self.R = R
        
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0
        
    def kalman(self,ADC_Value):
       
        self.Z_k = ADC_Value
        
        if (abs(self.kalman_adc_old-ADC_Value)>=0.005):
            self.x_k1_k1= ADC_Value*0.382 + self.kalman_adc_old*0.618
        else:
            self.x_k1_k1 = self.kalman_adc_old
    
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
    
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)
    
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
    
        self.kalman_adc_old = kalman_adc
        
        return kalman_adc

def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    
    # Concatenate higher and lower value
    value = ((high << 8) | low)
    
    # To get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.main_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.main_widget)
        layout = QtWidgets.QVBoxLayout(self.main_widget)

        self.graphWidget = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphWidget)

        self.setup_plots()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # Update every 50ms

        # Initialize Kalman filters
        self.kf_ax = kalman_filter(0.001, 0.1)
        self.kf_ay = kalman_filter(0.001, 0.1)
        self.kf_az = kalman_filter(0.001, 0.1)
        self.kf_gx = kalman_filter(0.001, 0.1)
        self.kf_gy = kalman_filter(0.001, 0.1)
        self.kf_gz = kalman_filter(0.001, 0.1)

    def setup_plots(self):
        self.p1 = self.graphWidget.addPlot(title="Accelerometer")
        self.p2 = self.graphWidget.addPlot(title="Gyroscope")
        self.graphWidget.nextRow()
        self.p3 = self.graphWidget.addPlot(title="Ax")
        self.p4 = self.graphWidget.addPlot(title="Ay")
        self.graphWidget.nextRow()
        self.p5 = self.graphWidget.addPlot(title="Az")
        self.p6 = self.graphWidget.addPlot(title="Gyro Magnitude")

        # Create curves
        self.curve_ax = self.p1.plot(pen='r', name='x')
        self.curve_ay = self.p1.plot(pen='g', name='y')
        self.curve_az = self.p1.plot(pen='b', name='z')
        self.curve_gx = self.p2.plot(pen='r', name='x')
        self.curve_gy = self.p2.plot(pen='g', name='y')
        self.curve_gz = self.p2.plot(pen='b', name='z')
        self.curve_ax_single = self.p3.plot(pen='r')
        self.curve_ay_single = self.p4.plot(pen='g')
        self.curve_az_single = self.p5.plot(pen='b')
        self.curve_g_mag = self.p6.plot(pen='w')

        # Initialize data arrays
        self.data_ax = np.zeros(100)
        self.data_ay = np.zeros(100)
        self.data_az = np.zeros(100)
        self.data_gx = np.zeros(100)
        self.data_gy = np.zeros(100)
        self.data_gz = np.zeros(100)

    def update(self):
        # Read sensor data
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)
        
        # Convert data
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0
        
        # Apply Kalman filter
        Ax = self.kf_ax.kalman(Ax)
        Ay = self.kf_ay.kalman(Ay)
        Az = self.kf_az.kalman(Az)
        Gx = self.kf_gx.kalman(Gx)
        Gy = self.kf_gy.kalman(Gy)
        Gz = self.kf_gz.kalman(Gz)
        
        # Update data arrays
        self.data_ax = np.roll(self.data_ax, -1)
        self.data_ax[-1] = Ax
        self.data_ay = np.roll(self.data_ay, -1)
        self.data_ay[-1] = Ay
        self.data_az = np.roll(self.data_az, -1)
        self.data_az[-1] = Az
        self.data_gx = np.roll(self.data_gx, -1)
        self.data_gx[-1] = Gx
        self.data_gy = np.roll(self.data_gy, -1)
        self.data_gy[-1] = Gy
        self.data_gz = np.roll(self.data_gz, -1)
        self.data_gz[-1] = Gz
        
        # Calculate gyro magnitude
        g_mag = np.sqrt(Gx**2 + Gy**2 + Gz**2)
        
        # Update plots
        self.curve_ax.setData(self.data_ax)
        self.curve_ay.setData(self.data_ay)
        self.curve_az.setData(self.data_az)
        self.curve_gx.setData(self.data_gx)
        self.curve_gy.setData(self.data_gy)
        self.curve_gz.setData(self.data_gz)
        self.curve_ax_single.setData(self.data_ax)
        self.curve_ay_single.setData(self.data_ay)
        self.curve_az_single.setData(self.data_az)
        self.curve_g_mag.setData(np.full_like(self.data_gx, g_mag))

if __name__ == '__main__':
    MPU_Init()
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec())