#! /usr/bin/env python

import rospy
import rosbag
import copy
import math
import sys
import os
import glob
import geodesy.utm
from gmplot_msgs.srv import PlotMap
from gmplot_msgs.msg import PlotPoint
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


class GmPlotter:
    def __init__(self, utm_zone=0):
        self.avg_lat = 0.0
        self.avg_lon = 0.0
        self.num_samples = 0
        self.dbw_enabled = False
        self.last_utm = geodesy.utm.UTMPoint()
        self.last_utm.z = 0
        self.last_utm.band = 'T'
        self.last_utm.zone = utm_zone
        self.plot_points = []

    def add_point(self, lat, lon):
        new_utm_position = geodesy.utm.fromLatLong(lat, lon)

        if self.dist2(new_utm_position, self.last_utm) > (1.0 * 1.0):
          self.avg_lat += lat
          self.avg_lon += lon
          self.num_samples += 1
          self.plot_points.append(PlotPoint(
              lat=lat,
              lon=lon,
              type=PlotPoint.LINE,
              size=5,
              color='g'
          ))
         
        self.last_utm = new_utm_position
     
    def add_utm_point(self, utm_x, utm_y):
        new_utm_position = geodesy.utm.UTMPoint()
        new_utm_position.easting = utm_x
        new_utm_position.northing = utm_y
        new_utm_position.z = 0
        new_utm_position.band = 'T'
        new_utm_position.zone = self.last_utm.zone
        if self.dist2(new_utm_position, self.last_utm) > (1.0 * 1.0):
          new_lat_lon = new_utm_position.toMsg()
          self.avg_lat += new_lat_lon.latitude
          self.avg_lon += new_lat_lon.longitude
          self.num_samples += 1
          self.plot_points.append(PlotPoint(
              lat=new_lat_lon.latitude,
              lon=new_lat_lon.longitude,
              type=PlotPoint.LINE,
              size=5,
              color='g'
          ))
          self.last_utm = new_utm_position
         

    @staticmethod
    def dist2(utm1, utm2):
      if math.isnan(utm1.easting) or math.isnan(utm2.easting):
          return float('inf')
      elif utm1.zone != utm2.zone:
          return float('inf')
      else: 
          return (utm1.easting - utm2.easting) * (utm1.easting - utm2.easting) + (utm1.northing - utm2.northing) * (utm1.northing - utm2.northing)

    def make_plot(self, filename):
        if self.num_samples == 0:
            rospy.logwarn('No samples to plot on Google map')
            return

        avg_lat = self.avg_lat / self.num_samples
        avg_lon = self.avg_lon / self.num_samples
        p1 = copy.deepcopy(self.plot_points[0])
        p2 = copy.deepcopy(self.plot_points[-1])
        if len(self.plot_points) > 1:
            self.plot_points.append(PlotPoint(
                lat=p1.lat,
                lon=p1.lon,
                type=PlotPoint.MARKER_WITH_TEXT,
                color='b',
                text='Start'
            ))
            self.plot_points.append(PlotPoint(
                lat=p2.lat,
                lon=p2.lon,
                type=PlotPoint.MARKER_WITH_TEXT,
                color='m',
                text='End'
            ))

        # Construct service request
        service = rospy.ServiceProxy('plot_google_map', PlotMap)
        num_tries = 0
        success = False
        while not success and num_tries < 10:
            try:
                rospy.loginfo('Calling gmplot service...')
                service.call(save_map=True,
                             center_lat=avg_lat,
                             center_lon=avg_lon,
                             zoom=16,
                             satellite_view=False,
                             filename=filename,
                             points=self.plot_points)
                success = True
                rospy.loginfo('Success!')
            except rospy.ServiceException as e:
                rospy.logwarn(str(e))
                num_tries += 1
                rospy.sleep(0.2)

        return success


class ProcessRawBag:
    def __init__(self):
        utm_zone = rospy.get_param('~utm_zone', default=17)

        bags = glob.glob('*.bag')
        print 'Bag files found: ' + str(len(bags))
        for fname in bags:
            print '    ' + fname

        plotter_service_exists = True

        for bag_path in bags:

            if rospy.is_shutdown():
                break

            gm_plotter = GmPlotter(utm_zone)
            rospy.loginfo('Processing bag file [%s]' % bag_path)

            try:
                bag = rosbag.Bag(bag_path)
                file_name = bag_path.split('.')[0]
                gps_topics = []
                topic_info = bag.get_type_and_topic_info()
                for i in topic_info.topics:
                    if topic_info.topics[i][0] == 'nav_msgs/Odometry' or topic_info.topics[i][0] == 'sensor_msgs/NavSatFix':
                        topics.append(i)


                for _, msg, _ in bag.read_messages(topics=gps_topics):
                    if msg._type == 'sensor_msgs/NavSatFix':
                        gm_plotter.add_point(msg.latitude, msg.longitude)
                    elif msg._type == 'nav_msgs/Odometry':
                        gm_plotter.add_utm_point(msg.pose.pose.position.x, msg.pose.pose.position.y)
                    else:
                        rospy.logwarn_once('Unsupported message type: %s' % msg._type)

                if rospy.is_shutdown():
                    break

                if plotter_service_exists:
                    plotter_service_exists = gm_plotter.make_plot(os.path.abspath(os.curdir) + '/' + file_name + '.html')
                    rospy.sleep(2.0)

                bag.close()
            except IOError, e:
                rospy.logerr(str(e))
            except OSError, e:
                rospy.logerr(str(e))
            except rosbag.ROSBagException, e:
                rospy.logerr('Unable to process bag file: ' + str(e))
                bag.close()
            except rospy.ROSInterruptException, e:
                rospy.loginfo(str(e))
                bag.close()


if __name__ == '__main__':
    rospy.init_node('process_raw_bag')
    try:
        ProcessRawBag()
    except rospy.ROSInterruptException:
        pass
