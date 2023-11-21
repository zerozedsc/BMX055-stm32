import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from collections import Counter
import matplotlib.pyplot as plt
import threading, os, math, sys

degree = { 0: {'x': [0.0], 'y': [0.0], 'z': [0.0]},
    30 : {'x': [0.0, -9.0, -17.0, -25.0, -1.0], 'y': [0.0], 'z': [0.0]},
          50 :  {'x': [-41.0, -57.0, -49.0, -33.0], 'y': [0.0], 'z': [0.0]},
80: {'x': [-65.0, -81.0, -89.0, -73.0], 'y': [0.0], 'z': [0.0]},
100: {'x': [-113.0, -97.0, -73.0, -121.0, -89.0, -81.0, -65.0, -105.0], 'y': [0.0], 'z': [0.0]},
110: {'x': [-105.0, -97.0, -113.0, -121.0], 'y': [-7.0, -23.0, -15.0, -31.0], 'z': [0.0]},
140: {'x': [-105.0, -113.0, -97.0, -121.0], 'y': [-63.0, -55.0, -39.0, -47.0], 'z': [0.0]},
160: {'x': [-81.0, -121.0, -73.0, -89.0, -113.0, -65.0, -105.0, -97.0], 'y': [-95.0, -87.0, -71.0, -79.0], 'z': [0.0]},
170: {'x': [-73.0, -81.0, -65.0, -89.0], 'y': [-95.0, -87.0, -71.0, -79.0], 'z': [0.0]},
180: {'x': [-73.0, -65.0, -81.0, -89.0], 'y': [-111.0, -103.0, -127.0, -119.0, -95.0, -87.0, -79.0], 'z': [0.0]},
190: {'x': [-73.0, -89.0, -65.0, -49.0, -57.0, -81.0, -41.0], 'y': [-119.0, -87.0, -127.0, -111.0, -103.0, -95.0], 'z': [0.0]},
200: {'x': [-57.0, -33.0, -41.0, -49.0], 'y': [-127.0, -103.0, -111.0, -119.0], 'z': [0.0]},
210: {'x': [-17.0, -25.0, -41.0, -49.0, -9.0, -1.0, -33.0], 'y': [-103.0, -119.0, -111.0, -127.0], 'z': [0.0]},
220: {'x': [-1.0, -9.0, -25.0, -17.0], 'y': [-127.0, -119.0, -111.0, -103.0], 'z': [0.0]},
240: {'x': [0.0], 'y': [-111.0, -119.0, -95.0, -87.0, -103.0, -127.0, -71.0, -79.0], 'z': [0.0]},
250: {'x': [0.0], 'y': [-79.0, -71.0, -95.0, -87.0], 'z': [0.0]},
270: {'x': [0.0], 'y': [-95.0, -63.0, -79.0, -71.0, -87.0, -39.0, -55.0, -47.0], 'z': [0.0]},
280: {'x': [0.0], 'y': [-39.0, -47.0, -55.0, -63.0], 'z': [0.0]},
290: {'x': [0.0], 'y': [-55.0, -39.0, -47.0, -63.0], 'z': [0.0]},
300: {'x': [0.0], 'y': [-47.0, -55.0, -7.0, -31.0, -63.0, -39.0, -23.0, -15.0], 'z': [0.0]},
310: {'x': [0.0], 'y': [-15.0, -31.0, -23.0, -7.0], 'z': [0.0]},
330: {'x': [0.0], 'y': [-7.0, -31.0, -15.0, 0.0, -23.0], 'z': [0.0]},
340: {'x': [0.0], 'y': [0.0, -15.0, -7.0, -23.0], 'z': [0.0]},
350: {'x': [0.0], 'y': [0.0], 'z': [0.0]},
380: {'x': [0.0], 'y': [0.0], 'z': [0.0]}
}
# repeated data collected (freq > 10) check for every 500 data continues reading from sensor, 
#between 30-50, all same with 30 same with betwwen 80-50, all same with 50, same with before



average_degree100 ={ 0: {'x': 0.0, 'y': 0.0, 'z': 0.0},
                 10: {'x': -0.7747747747747747, 'y': 0.0, 'z': 0.0},
                 20: {'x': -6.392156862745098, 'y': 0.0, 'z': 0.0},
                 30: {'x': -10.851851851851851, 'y': 0.0, 'z': 0.0},
                 40: {'x': -13.277227722772277, 'y': 0.0, 'z': 0.0},
                 50: {'x': -43.88, 'y': 0.0, 'z': 0.0},
                 60: {'x': -45.78350515463917, 'y': 0.0, 'z': 0.0},
                 70: {'x': -72.13725490196079, 'y': 0.0, 'z': 0.0},
                 80: {'x': -75.96, 'y': 0.0, 'z': 0.0},
                 90: {'x': -79.72, 'y': 0.0, 'z': 0.0},
                 100: {'x': -95.4, 'y': -3.01, 'z': 0.0},
                 110: {'x': -110.2, 'y': -15.73, 'z': 0.0},
                 120: {'x': -109.0, 'y': -28.12, 'z': 0.0},
                 130: {'x': -108.28, 'y': -50.92, 'z': 0.0},
                 140: {'x': -109.56, 'y': -53.72, 'z': 0.0},
                 150: {'x': -109.4, 'y': -76.68, 'z': 0.0},
                 160: {'x': -109.4, 'y': -84.92, 'z': 0.0},
                 170: {'x': -104.44, 'y': -101.0, 'z': 0.0},
                 180: {'x': -78.12, 'y': -116.68, 'z': 0.0},
                 190: {'x': -73.88, 'y': -113.56, 'z': 0.0},
                 200: {'x': -45.72, 'y': -123.96, 'z': 0.0},
                 210: {'x': -45.56, 'y': -137.88, 'z': 0.0},
                 220: {'x': -13.8, 'y': -142.12, 'z': 0.0},
                 230: {'x': -13.4, 'y': -140.2, 'z': 0.0},
                 240: {'x': -2.25, 'y': -128.04, 'z': 0.0},
                 250: {'x': 0.0, 'y': -116.52, 'z': 0.0},
                 260: {'x': 0.0, 'y': -115.48, 'z': 0.0},
                 270: {'x': 0.0, 'y': -85.24, 'z': 0.0},
                 280: {'x': 0.0, 'y': -82.36, 'z': 0.0},
                 290: {'x': 0.27, 'y': -63.32, 'z': 0.0},
                 300: {'x': 1.62, 'y': -48.92, 'z': 0.0},
                 310: {'x': 0.83, 'y': -36.68, 'z': 0.0},
                 320: {'x': 0.0, 'y': -18.28, 'z': 0.0},
                 330: {'x': 0.0, 'y': -19.4, 'z': 0.0},
                 340: {'x': 0.0, 'y': -11.68, 'z': 0.0},
                 350: {'x': 0.0, 'y': -0.07, 'z': 0.0},
                 360: {'x': 0.0, 'y': 0.0, 'z': 0.0}
}
 # average data collected check for every 100 data continues reading from sensor, 

average_degree10 = {
    0: {'x': 0.0, 'y': 0.0, 'z': 0.0},
    10: {'x': 0.0, 'y': 1.8, 'z': 0.0},
    20: {'x': -2.0, 'y':3.2, 'z': 0.0},
    30: {'x': -9.8, 'y': 0.0, 'z': 0.0},
    40: {'x': -18.0, 'y': 5.2, 'z': 0.0},
    50: {'x': -42.0, 'y': 3.2, 'z': 0.0},
    60: {'x': -46.8, 'y': 6.4, 'z': 0.0},
    70: {'x': -52.8, 'y': 5.6, 'z': 0.0},
    80: {'x': -75.4, 'y': 0.0, 'z': 0.0},
    90: {'x': -79.2, 'y': -2.2, 'z': 0.0},
    100: {'x': -98.0, 'y': 0.0, 'z': 0.0},
    110: {'x': -110.0, 'y': -4.4, 'z': 0.0},
    120: {'x': -114.8, 'y': -19.8, 'z': 0.0},
    130: {'x': -111.6, 'y': -34.2, 'z': 0.0},
    140: {'x': -114.8, 'y': -46.2, 'z': 0.0},
    150: {'x': -115.4, 'y': -59.8, 'z': 0.0},
    160: {'x': -114.8, 'y': -76.6, 'z': 0.0},
    170: {'x': -98.0, 'y': -86.2, 'z': 0.0},
    180: {'x': -80.0, 'y': -100.2, 'z': 0.0},
    190: {'x': -71.6, 'y': -110.8, 'z': 0.0},
    200: {'x': -47.4, 'y': -115.0, 'z': 0.0},
    210: {'x': -41.2, 'y': -119.8, 'z': 0.0},
    220: {'x': -20.4, 'y': -115.0, 'z': 0.0},
    230: {'x': -14.2, 'y': -113.4, 'z': 0.0},
    240: {'x': -5.6, 'y': -119.0, 'z': 0.0},
    250: {'x': 0.0, 'y': -110.2, 'z': 0.0},
    260: {'x': 0.0, 'y': -103.0, 'z': 0.0},
    270: {'x': 0.0, 'y': -85.8, 'z': 0.0},
    280: {'x': 0.0, 'y': -79.8, 'z': 0.0},
    290: {'x': 0.0, 'y': -60.0, 'z': 0.0},
    300: {'x': 0.0, 'y': -57.0, 'z': 0.0},
    310: {'x': 0.0, 'y': -45.0, 'z': 0.0},
    320: {'x': 0.0, 'y': -36.0, 'z': 0.0},
    330: {'x': 0.0, 'y': -14.4, 'z': 0.0},
    340: {'x': 0.0, 'y': -8.4, 'z': 0.0},
    350: {'x': 0.0, 'y': -1.8, 'z': 0.0},
}

def clamp(min, max, val):
    if val > max:
        return max
    
    if val < min:
        return min
    
    return val

class BMX055Subscriber(Node):
    def __init__(self):
        super().__init__("bmx055_subscriber")
        POS_LIST = [0]*3
        self.RAW = {"gyro" : POS_LIST, "accel": POS_LIST, "mag": POS_LIST}
        self.HALF_RAW = {"gyro_pos" : POS_LIST, "gyro_movement" : POS_LIST, 
                    "accel": POS_LIST, "velocity": POS_LIST, 
                    "mag_pos": POS_LIST,
                    "gyro_offset" : POS_LIST, "accel_offset" : POS_LIST}
        self.SAVE_DATA = {}
        
        self.gyro = {"x":[], "y":[], "z":[]}
        self.accel = {"x":[], "y":[], "z":[]}
        self.mag = {"x":[], "y":[], "z":[]}

        #save variable

        self.prevMag = 0
        self.newMag = 0

        # Create a QoS profile with best_effort reliability
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

        # Use the QoS profile for the subscription
        self.bmx055_raw = self.create_subscription(Float32MultiArray, "/BMX055_RAW", self.rawCallback, qos)
        # self.bmx055_data = self.create_subscription(Float32MultiArray, "/BMX055_DATA", self.dataCallback, qos)

    def rawCallback(self, msg):
        os.system("clear")
        keys = list(self.RAW.keys())

        c = 0
        for key in keys:
            self.RAW[key] = [float(msg.data[i]) for i in range(c, c+3)]
            c+=3
        

        self.checkSaveDataCoor(keys)

        sensor = [(a, self.SAVE_DATA[a]) for a in keys]
        datas = [(a, self.RAW[a]) for a in keys]
        heading = 0
        headingDegree = 0

        for name, data in sensor:
            x,y,z = self.SAVE_DATA[name].values()
            if len(x) != 0 and len(y) != 0 and len(z) != 0:
                #print(f"{name} -> x[{len(x)}]:{sum(abs(a) for a in x)/len(x)} y[{len(y)}]:{sum(abs(b) for b in y)/len(y)} z[{len(z)}]:{sum(abs(c) for c in z)/len(z)}")
                print(f"{name} -> x[{len(x)}]:{max(x)}${min(x)} y[{len(y)}]:{max(y)}${min(y)} z[{len(z)}]:{max(z)}${min(z)}")
                if name == "mag":
                    self.checkMag(x,y,z,1, data=data)
                

        self.setupGraph(sensor, datas)
        # sprint(self.RAW)

    
    def checkMag(self,x:list,y:list,z:list,mode:int,data:{}={}):

        def closestDegree(average_values, data, threshold):
            min_difference = float("inf")  # Set an initial minimum distance to infinity
            matching_key = None

            for key, values in average_values.items():
                # Calculate absolute differences for each coordinate (x, y, z)
                difference_x = abs(values["x"] - data["x"])
                difference_y = abs(values["y"] - data["y"])
                difference_z = abs(values["z"] - data["z"])

                # Calculate the total difference
                total_difference = difference_x + difference_y + difference_z

                # Check if the total difference is within a certain threshold
                if total_difference < threshold:  # Adjust this threshold as needed
                    matching_key = key
                    break  # Break the loop if a match is found within the threshold
                elif total_difference < min_difference:
                    min_difference = total_difference
                    matching_key = key
            
            return matching_key


        if mode == 0:
            result_average = {"x": sum(float(a) for a in x[-10:])/10, 
                                "y": sum(float(a) for a in y[-10:])/10,
                                "z": sum(float(a) for a in z[-10:])/10}
                        
            #print("repeated data: ", result)
            print("average data: ", result_average)


        if mode == 1 and (len(x) > 10):        
            result_average = {"x": sum(float(a) for a in x[-10:])/10, 
                                "y": sum(float(a) for a in y[-10:])/10,
                                "z": sum(float(a) for a in z[-10:])/10}
                            
            #print("repeated data: ", result)
            print("average data of last 10: ", result_average)
            print("closest degree: ", closestDegree(average_degree10, result_average, 5))



        

    def dataCallback(self, msg):
        keys = list(self.HALF_RAW.keys())
        c = 0
        for key in keys:
            self.HALF_RAW[key] = [float(msg.data[i]) for i in range(c, c+3)]
            c+=3

        # sensor = [(keys[0], self.gyro), (keys[1], self.accel), (keys[2], self.mag)]
        # data = [(keys[0], self.HALF_RAW[keys[0]]), (keys[1], self.HALF_RAW[keys[1]]), (keys[2], self.HALF_RAW[keys[2]])]
          
                
        # self.setupGraph(sensor, data)
        
        #print(self.HALF_RAW)

    def setupGraph(self, save:list, data:list):
        
        for name, k in save:
            for nm, b in data:
                if nm == name:
                    k["x"].append(b[0])
                    k["y"].append(b[1])
                    k["z"].append(b[2])
                    break

                
        self.plotGraph(save)
    

    def checkSaveDataCoor(self, keys):
        for key in keys:
            if key not in self.SAVE_DATA:
                self.SAVE_DATA[key] = {"x":[], "y":[], "z":[]}


    def plotGraph(self, data:list):
        
        plt.ion()

        plt.clf()

        len_data = len(data)
        

        for k in range(len_data):
            keys = list(data[k][1].keys())
            plt.subplot(len_data, 1, k+1)
            plt.plot(data[k][1]["x"], label=f"{data[k][0]} X")
            plt.plot(data[k][1]["y"], label=f"{data[k][0]} Y")
            plt.plot(data[k][1]["z"], label=f"{data[k][0]} Z")
            plt.title(f"{data[k][0]} Data")
            plt.legend()


        plt.pause(0.001)
            


def main(args=None):
    rclpy.init(args=args)
    sub = BMX055Subscriber()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()