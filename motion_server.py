# Description: This is a simple server that listens for POST requests and prints the message to the console.
# Future Work: Upgrade to a web socket server to allow for real-time communication between the server and the client.
# Import Flask
from flask import Flask, request
from flask_cors import CORS, cross_origin
import urllib.parse
import json
from flask.logging import default_handler

import rospy 
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Create an app instance
app = Flask(__name__)
cors = CORS(app, methods="*", allow_headers="Content-Type")
# cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
app.logger.removeHandler(default_handler)
joystick_left_publisher = rospy.Publisher('/VR/joy_left', Joy, queue_size=1)
joystick_right_publisher = rospy.Publisher('/VR/joy_right', Joy, queue_size=1)
joystick_mode_publisher = rospy.Publisher('/VR/joy_mode', String, queue_size=1)
rospy.init_node('motion_server', anonymous=True)

#Store previous x and y values
prev_left_x = 0.0
prev_left_y = 0.0
prev_right_x = 0.0
prev_right_y = 0.0
# Define a route for POST requests
@app.route('/', methods=['POST'])
@cross_origin()
def receive_message():
    # Get the message from the request body
    # message = request.get_data().decode('utf-8')
    data = request.get_data()
    message = urllib.parse.unquote_plus(data.decode('utf-8'))
    ##print("msg: ", message)
    data = json.loads(message)
    ##print("data: ", data)
    if data is None:
        ##print("No data receved")
        return "No data received", 400
    try:
        content = data['content']
        # print(f'Received message: {message}, with data: {content}')
        
        mode = content['Mode']
        left_joy = content['leftJoystick']
        right_joy = content['rightJoystick']
        publish_joy(mode, left_joy, right_joy)
        return content, 200
    except:
        print("invalid shit")
        return "Invalid data received", 400

def publish_joy(mode, left_joy, right_joy):
    # data = (x,y)
    try:
        send = True
        left_x = float(left_joy['x'])
        left_y = float(left_joy['y'])
        right_x = float(right_joy['x'])
        right_y = float(right_joy['y']) 
        left_joy_msg = Joy()
        left_joy_msg.header.stamp = rospy.Time.now()

        # if left_x == prev_left_x & left_y == prev_left_y:
        #     left_x = 0.0
        #     left_y = 0.0

        left_joy_msg.axes = [left_x,left_y,0, 0, 0,]
        # prev_left_x = left_x
        # prev_left_y = left_y

        right_joy_msg = Joy()
        right_joy_msg.header.stamp = rospy.Time.now()     

        # if right_x == prev_right_x & right_y == prev_right_y:
        #     right_x = 0.0
        #     right_y = 0.0

        right_joy_msg.axes = [right_x,right_y,0, 0, 0,]
        # prev_right_x = right_x
        # prev_right_y = right_y
        
        mode_out = String()

        
        ## Mode: 0 == standing
        ## Mode: 1 == walking
        if int(mode) == 0:
            mode_out.data = "standing"
        elif int(mode) == 1:
            mode_out.data = "walking"
        else:
            send = False
            
        if(send == False):
            # print("Invalid data received")
            return
        
        print(f'Left Joy: {left_x}, {left_y}')
        print(f'Right Joy: {right_x}, {right_y}')
        print(f'Mode: {mode_out.data}')
        
        
        joystick_left_publisher.publish(left_joy_msg)
        joystick_right_publisher.publish(right_joy_msg)
        joystick_mode_publisher.publish(mode_out)
    except Exception as e:
        # print("Error in parsing data: ", e)
        return e


# Run the app
if __name__ == '__main__':
    print("starting server...")
    app.run(debug=False, host="0.0.0.0")



