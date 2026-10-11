#!/usr/bin/env python
"""Start rear USB capture only after a fresh, confirmed path-loss handoff."""
import json
import threading


class DeferredRearCamera(object):
    def __init__(self,start,started_at,timeout=1.25):
        self.start=start
        self.started_at,self.timeout=started_at,timeout
        self.lock=threading.Lock()
        self.latest=None
        self.started=False
        self.launch=None

    def observe(self,data):
        with self.lock:self.latest=data

    def tick(self,now):
        with self.lock:data=self.latest
        if self.started or not isinstance(data,dict):return
        try:
            stamp=float(data['stamp'])
            requested=(data.get('state')=='PARKING' and data.get('action')=='PARKING'
                and data.get('enabled') is True and data.get('action_source')=='turn_path_loss'
                and (data.get('parking_entry') or {}).get('camera_source')=='rear')
            if not requested or stamp<self.started_at or not 0<=now-stamp<=self.timeout:return
        except (KeyError,ValueError,TypeError,AttributeError):
            return
        # Called by the main thread: roslaunch owns signal handlers.
        self.launch=self.start()
        self.started=True

    def shutdown(self):
        if self.launch is not None:self.launch.shutdown()


def main():
    import os
    import rospy
    import rospkg
    import roslaunch
    from std_msgs.msg import String
    rospy.init_node('rear_camera_gate')
    path=os.path.join(rospkg.RosPack().get_path('robocup_competition'),
                      'launch','rear_camera_capture.launch')
    args=['rear_device:='+rospy.get_param('~rear_device'),
          'rear_fps:='+str(rospy.get_param('~rear_fps',8)),
          'start_ground:='+str(rospy.get_param('~start_ground',True)).lower()]
    def start():
        uuid=roslaunch.rlutil.get_or_generate_uuid(None,False)
        roslaunch.configure_logging(uuid)
        launch=roslaunch.parent.ROSLaunchParent(uuid,[(path,args)])
        try:launch.start()
        except Exception:
            launch.shutdown()
            raise
        rospy.loginfo('rear_camera_start: confirmed turn_path_loss; starting rear USB and BEV')
        return launch
    camera=DeferredRearCamera(start,rospy.Time.now().to_sec())
    def status(msg):
        try:camera.observe(json.loads(msg.data))
        except (ValueError,TypeError):pass
    rospy.Subscriber('/competition/status',String,status,queue_size=1)
    rospy.on_shutdown(camera.shutdown)
    rospy.loginfo('rear_camera_wait: USB capture remains OFF until confirmed turn_path_loss')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.tick(rospy.Time.now().to_sec())
        if camera.launch is not None:camera.launch.spin_once()
        rate.sleep()


if __name__=='__main__':main()
