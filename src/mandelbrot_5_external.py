#Physical Bot
#import mandelbrot_movement_1 as mdlm

#GPS location and IoT sensing
#import mandelbrot_location_1 as mdls
#import compass_1 as cmp

#Vision and Distance
#import mandelbrot_vision_7 as mdlv

#OS and controls
import curses

#GIS
#import bot_map_new14 as btm

def main(window):
    print("Here.................")
    next_key = None
    motion = 0
        
    while True:

        if next_key is None:
            key = window.getch()
        else:
            key = next_key
        if key != -1:
            # KEY PRESSED
            if key == 259:  #Arrow up 
                print("bot_mv.forward()\r")
                motion = 1
                    
            if key == 260:  #Left arrow
                print("bot_mv.left()\r")
                motion = 1

            if key == 261:  #Right arrow
                print("bot_mv.right()\r")
                motion = 1
 
            if key == 258:  #Down arrow
                print("bot_mv.stop()\r")
                motion = 0
                
            if key == 114:  #R key
                print("bot_mv.reverse()\r")
                motion = 1

            curses.halfdelay(1)
                #print("Temperature", cmp.mpu.readTemperatureMaster())
                #print("\r")            
            next_key = None
            if motion == 1:
               print("Location Here\r")
                    #gps_location = mdls.gps.ubx_PVT()
                    #print("Latitude / Longitude / Accuracy(mm)", gps_location.lat, gps_location.lon, gps_location.hAcc)
               print("Coordinates Here\r")                
                    #print("Accelerometer", cmp.mpu.readAccelerometerMaster())
               print("Acceleration Here\r")                
                    #print("Gyro", cmp.mpu.readGyroscopeMaster())
               print("Gyro Here\r")                
                    #direction_angle = cmp.to_degrees(cmp.mpu.readMagnetometerMaster())
                    #print("Magnometer (heading)", direction_angle)
               print("Magnometer Here\r")
                    #print("Temperature", cmp.mpu.readTemperatureMaster())
               print("Temperature Here\r")
               print("\r")
               print("\r")
               print("\r")
            if motion == 0:
               print("Temperature Here\r")
               print("\r")
               print("\r")  
               print("\r")               
                 

if __name__ == '__main__':
    print("Here.................")
    curses.wrapper(main)


    