#! /usr/bin/env python

import rospy
from gmplot_msgs.msg import PlotPoint
from gmplot_msgs.srv import PlotMap, PlotMapRequest, PlotMapResponse

from gmplot import GoogleMapPlotter


class PlotGroup:
    def __init__(self, first_point):
        self.size = first_point.size
        self.color = first_point.color
        self.type = first_point.type
        self.members = [first_point]

    def add_to_group(self, plot_point):
        if plot_point.type == self.type and plot_point.size == self.size and plot_point.color == self.color:
            self.members.append(plot_point)
            return True
        else:
            return False


class GmplotService:
    def __init__(self):
        self.srv = rospy.Service('plot_google_map', PlotMap, self.service_cb)

    def service_cb(self, request):

        # request = PlotMapRequest()

        if len(request.points) == 0:
            raise rospy.ServiceException('No points present in request')

        # Sort points into groups of like types
        plot_groups = []

        for p in request.points:
            added_to_group = False
            for group in plot_groups:
                added_to_group = group.add_to_group(p)

            if not added_to_group:
                plot_groups.append(PlotGroup(p))

        gplot = GoogleMapPlotter(request.center_lat, request.center_lon, request.zoom)

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
            elif group.type == PlotPoint.TEXT_LABEL:
                for i in xrange(len(group.members)):
                    gplot.text(lats[i], lons[i], colors[i], text=texts[i])
            else:
                raise rospy.ServiceException('Tried to plot an unsupported type')

        gplot.draw(request.filename)

        return PlotMapResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('gmplot_service')
        node_instance = GmplotService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
