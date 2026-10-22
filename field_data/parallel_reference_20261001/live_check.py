from __future__ import print_function
import subprocess,signal,time,json,threading,os
import rospy,cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
directory='/home/nano/robocup_ws/field_data/parallel_reference_20261001'
if not os.path.isdir(directory):os.makedirs(directory)
log=open(directory+'/launch.log','w')
proc=subprocess.Popen(['roslaunch','robocup_competition','parallel_reference_camera.launch'],stdout=log,stderr=log)
rows=[];lock=threading.Lock();bridge=CvBridge();saved=[False];capture_started=time.time();late_saved=[False]
def reference(msg):
    data=json.loads(msg.data);data['received_age_s']=rospy.Time.now().to_sec()-data['stamp']
    with lock:rows.append(data)
def image(msg):
    if not saved[0]:
        saved[0]=True
        cv2.imwrite(directory+'/front.png',bridge.imgmsg_to_cv2(msg,'bgr8'))
    elif not late_saved[0] and time.time()-capture_started>6:
        late_saved[0]=True
        cv2.imwrite(directory+'/late_front.png',bridge.imgmsg_to_cv2(msg,'bgr8'))
try:
    rospy.init_node('p1_reference_read_only_check',anonymous=True)
    rospy.Subscriber('/parallel_parking/p1_reference',String,reference,queue_size=100)
    rospy.Subscriber('/front/usb_cam/image_raw',Image,image,queue_size=1)
    time.sleep(8)
finally:
    proc.send_signal(signal.SIGINT);proc.wait();log.close()
with open(directory+'/observations.json','w') as out:json.dump(rows,out,indent=2)
for row in rows[:3]+rows[-3:]:
    print(json.dumps({k:row.get(k) for k in ('stamp','reason','motion_valid','p1_lines','inliers','received_age_s')}))
print('observations',len(rows))
