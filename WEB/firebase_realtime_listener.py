#!/usr/bin/env python

# firebase connect
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# init api_key to connect with firebase 
import os
current_path = os.path.dirname(os.path.realpath(__file__))
cred = credentials.Certificate("{}/firebase_key.json".format(current_path))
app = firebase_admin.initialize_app(cred)
db = firestore.client()


# create an Event for notifying main thread
import threading
callback_done = threading.Event()

# ROS
import rospy


class Realtime_listener:
    def __init__(self):
        rospy.init_node("realtime_listener", anonymous=True)
        self.point_pub = rospy.Publisher('/point',Float64MultiArray, queue_size=1)
        
        # you can set a specific location like this...
        doc_ref = db.collection(u'Ego').document(u'point_list')
        # start realtime listener at doc_Ref
        doc_watch = doc_ref.on_snapshot(self.on_snapshot)
        

        self.change_type = False
        self.check_time = ""
        self.data = {}

        rate = rospy.Rate(2) # 2 times / 1 sec
        while not rospy.is_shutdown():
            if self.change_type == 'MODIFIED' or self.change_type == 'ADDED':
                print("-----------------------")
                print("modified at {} !!".format(self.check_time))
                print("doc_ref data : {}".format(self.data))
                print("-----------------------")
                
                point_msg = Float64MultiArray()

                point_msg.data = [self.data.get('start_position_x', 0), self.data.get('start_position_y', 0), self.data.get('pinpoint01_x', 0), self.data.get('pinpoint01_y', 0), self.data.get('end_position_x', 0), self.data.get('end_position_y', 0)]

                '''
                point_msg.layout.dim.append(MultiArrayDimension())
                point_msg.layout.dim.append(MultiArrayDimension())
                point_msg.layout.dim[0].label = "height"
                point_msg.layout.dim[1].label = "width"
                point_msg.layout.dim[0].size = 2  # 2 rows
                point_msg.layout.dim[1].size = 2  # 2 columns
                point_msg.layout.dim[0].stride = 4
                point_msg.layout.dim[1].stride = 1
                point_msg.layout.data_offset = 0

                point_msg.data = [
                    [self.data.get('start_position_x', 0),
                    self.data.get('start_position_y', 0)],
                    [self.data.get('end_position_x', 0),
                    self.data.get('end_position_y', 0)]
                ]
                '''

                self.point_pub.publish(point_msg)


            else:
                print('no changed...')

            

            self.change_type = False
            rate.sleep()


    # firebase reference (data type and fields)  >>  https://cloud.google.com/firestore/docs/reference/rpc/google.firestore.v1#documentchange
    # this function works only when there is a change at doc_Ref
    def on_snapshot(self, doc_snapshot, changes, read_time):
        
        # change type : 'ADDED', 'MODIFIED', 'REMOVED'
        for change in changes:
            self.change_type = change.type.name

        # you can check data here
        for doc in doc_snapshot:
            self.data =  doc.to_dict()

        # read time (based on the region where your DB is located)
        self.check_time = read_time

        callback_done.set()




if __name__ == "__main__":
    try:
        start = Realtime_listener()
    except rospy.ROSInterruptException:
        pass