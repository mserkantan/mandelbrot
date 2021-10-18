import cv2
from flask import Flask, Response, render_template
from random import randint

app = Flask(__name__)
video = cv2.VideoCapture(0)

@app.route('/')
def index():
    return render_template('dashboard-7.html')
             
def Capture_data(video):
    font = cv2.FONT_HERSHEY_SIMPLEX
    size = 2
    color = (0, 255, 255)
    thickness = 2
    
    timer = 0
    message = "Start"
   
    while True:
        success, image = video.read()
        timer = timer + 1

        if timer == 100:
            dummy_output = randint(0, 10)
            timer = 0
            if dummy_output > 7:
                ###### We need a way to export this one string below to the main program ############
                message = str(dummy_output)

        cv2.putText(image, message, (50, 50), font, size, color, thickness, cv2.LINE_4)        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]        
        ret, jpeg2 = cv2.imencode('.jpg', image, encode_param)
        frame = jpeg2.tobytes()
        
 

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
      return Response(Capture_data(video),
             mimetype='multipart/x-mixed-replace; boundary=frame')  


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=2204, threaded=True)