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
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'
app.logger.removeHandler(default_handler)
joystick_left_publisher = rospy.Publisher('/VR/joy_left', Joy, queue_size=1)
joystick_right_publisher = rospy.Publisher('/VR/joy_right', Joy, queue_size=1)
joystick_mode_publisher = rospy.Publisher('/VR/joy_mode', String, queue_size=1)
rospy.init_node('motion_server', anonymous=True)

# Define a route for POST requests
@app.route('/', methods=['POST'])
@cross_origin()
def receive_message():
    # Get the message from the request body
    # message = request.get_data().decode('utf-8')
    data = request.get_data()
    message = urllib.parse.unquote_plus(data.decode('utf-8'))

    data = json.loads(message)
    content = data['content']
    # print(f'Received message: {message}, with data: {content}')
    
    # Print the message to the console
    # print(f'Received message: {message}, with data: {content}')
    publish_joy(content)
    # Return a response
    # return 'OK Homie, we da music', 200
    return content, 200

def publish_joy(data):
    # data = (x,y)
    if data is not None:
        data = data.split(',')
        try:
            x = data[0][1:]
            z = data[1][:-1]
            joy_msg = Joy()
            joy_msg.header.stamp = rospy.Time.now()
            # logging.error("X: ", x, " Z: ", z)
            print("X: ", x, " Z: ", z)
            joy_msg.axes = [float(z), 0,0, 0, 0, float(x)]
            # print("Publishing Joy Message: ", joy_msg.axes)
            joystick_left_publisher.publish(joy_msg)
            joystick_right_publisher.publish(joy_msg)
            joystick_mode_publisher.publish(joy_msg)
        except:
            print("Error in parsing data")


# Run the app
if __name__ == '__main__':
    print("starting server...")
    app.run(debug=False, host="0.0.0.0")



