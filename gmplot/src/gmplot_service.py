#! /usr/bin/env python

import rospy
import tempfile
import os
from gmplot_msgs.msg import PlotPoint
from gmplot_msgs.srv import PlotMap, PlotMapRequest, PlotMapResponse

from gmplot import GoogleMapPlotter


# This class orgainizes groups of markers that should be plotted together
class PlotGroup:
    def __init__(self, first_point):
        self.size = first_point.size
        self.color = first_point.color
        self.type = first_point.type
        self.members = [first_point]
        self.accepting_new_members = True

    # Only add members if type, size, and color match,
    # and if there hasn't been a break in the matches across
    # sequential calls to this method
    def add_to_group(self, plot_point):
        if plot_point.type == self.type \
                and plot_point.size == self.size\
                and plot_point.color == self.color \
                and self.accepting_new_members:
            self.members.append(plot_point)
            return True
        else:
            self.accepting_new_members = False
            return False


# ROS node class that advertises the plot service
class GmplotService:

    MIN_TIME_BETWEEN_CALLS = 3.0

    def __init__(self):
        rospy.init_node('gmplot_service')
        self.srv = rospy.Service('plot_google_map', PlotMap, self.service_cb)

        # Create a temporary file to store maps meant only for display
        self.tmp = tempfile.NamedTemporaryFile(suffix='.html')

        self.last_called_stamp = rospy.Time(0)

    # This runs once after the node has started to shut down
    def shutdown_handler(self):
        # Close and delete temporary map file
        self.tmp.close()

    # Service callback
    def service_cb(self, request):
        dt = (rospy.Time.now() - self.last_called_stamp).to_sec()
        if dt < self.MIN_TIME_BETWEEN_CALLS:
            raise rospy.ServiceException("Minimum time between plot requests is %f seconds.  It's only been %f seconds" % (self.MIN_TIME_BETWEEN_CALLS, dt))

        if len(request.points) == 0:
            raise rospy.ServiceException('No points present in request')

        # Sort points into sequential groups of like types
        plot_groups = []
        for p in request.points:
            added_to_group = False
            # See if next point belongs in an existing group
            for group in plot_groups:
                added_to_group = group.add_to_group(p)

            # Create new plotting group if current point hasn't been added
            # to an existing one
            if not added_to_group:
                plot_groups.append(PlotGroup(p))

        # Initialize Google Maps plot
        map_type = 'satellite' if request.satellite_view else None
        gplot = GoogleMapPlotter(request.center_lat, request.center_lon, request.zoom, map_type=map_type)

        # Loop through plotting groups to add plots to map
        for group in plot_groups:
            lats = [p.lat for p in group.members]
            lons = [p.lon for p in group.members]
            colors = [p.color for p in group.members]
            texts = [p.text for p in group.members]

            if group.type == PlotPoint.SCATTER_POINT:
                gplot.scatter(lats, lons, group.color, size=group.size, marker=False)
            elif group.type == PlotPoint.LINE:
                gplot.plot(lats, lons, group.color, edge_width=group.size)
            elif group.type == PlotPoint.MARKER:
                gplot.scatter(lats, lons, group.color, marker=True)
            elif group.type == PlotPoint.TEXT_LABEL or group.type == PlotPoint.MARKER_WITH_TEXT:
                for i in xrange(len(group.members)):
                    gplot.text(lats[i], lons[i], colors[i], text=texts[i], marker=(group.type == PlotPoint.MARKER_WITH_TEXT))
            else:
                raise rospy.ServiceException('Tried to plot an unsupported type')

        self.last_called_stamp = rospy.Time.now()
        if request.save_map:
            # Just write map to designated file and be done
            gplot.draw(request.filename)
        else:
            # Write the map to the temporary file and immediately open default
            # browser to view it
            gplot.draw(self.tmp.name)
            os.system('xdg-open ' + self.tmp.name)

        return PlotMapResponse()


if __name__ == '__main__':
    try:
        node_instance = GmplotService()
        rospy.on_shutdown(node_instance.shutdown_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
