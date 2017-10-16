import math
import os

from color_dicts import mpl_color_map, html_color_codes


def safe_iter(var):
    try:
        return iter(var)
    except TypeError:
        return [var]


class GoogleMapPlotter(object):

    def __init__(self, center_lat, center_lng, zoom, map_type=None):
        self.center = (float(center_lat), float(center_lng))
        self.zoom = int(zoom)
        self.grids = None
        self.paths = []
        self.shapes = []
        self.points = []
        self.text_points = []
        self.coloricon = os.path.join(os.path.dirname(__file__), 'markers/%s.png')
        self.coloricon = self.coloricon.replace('\\', '\\\\')
        self.color_dict = mpl_color_map
        self.html_color_codes = html_color_codes

        # option for satellite map, default is road map
        if map_type == ('satellite' or 'Satellite' or 'SATELLITE'):
            self.map_type = 'google.maps.MapTypeId.SATELLITE'
        else:
            self.map_type = 'google.maps.MapTypeId.ROADMAP'

    def marker(self, lat, lng, color='#FF0000', c=None, title="no implementation"):
        if c:
            color = c
        color = self.color_dict.get(color, color)
        color = self.html_color_codes.get(color, color)
        self.points.append((lat, lng, color[1:], title))

    def text(self, lat, lng, color='#000000', c=None, text="no implementation", marker=False):
        if c:
            color = c
        color = self.color_dict.get(color, color)
        color = self.html_color_codes.get(color, color)
        self.text_points.append((lat-5e-5, lng, color[1:], text))
        if marker:
            self.marker(lat, lng, color)

    def scatter(self, lats, lngs, color=None, size=None, marker=True, c=None, s=None, **kwargs):
        color = color or c
        size = size or s or 40
        kwargs["color"] = color
        kwargs["size"] = size
        settings = self._process_kwargs(kwargs)
        for lat, lng in zip(lats, lngs):
            if marker:
                self.marker(lat, lng, settings['color'])
            else:
                self.circle(lat, lng, size, **settings)

    def circle(self, lat, lng, radius, color=None, c=None, **kwargs):
        color = color or c
        kwargs.setdefault('face_alpha', 0.5)
        kwargs.setdefault('face_color', "#000000")
        kwargs.setdefault("color", color)
        settings = self._process_kwargs(kwargs)
        path = self.get_cycle(lat, lng, radius)
        self.shapes.append((path, settings))

    def _process_kwargs(self, kwargs):
        settings = dict()
        settings["edge_color"] = kwargs.get("color", None) or \
                                 kwargs.get("edge_color", None) or \
                                 kwargs.get("ec", None) or \
                                 "#000000"

        settings["edge_alpha"] = kwargs.get("alpha", None) or \
                                 kwargs.get("edge_alpha", None) or \
                                 kwargs.get("ea", None) or \
                                 1.0
        settings["edge_width"] = kwargs.get("edge_width", None) or \
                                 kwargs.get("ew", None) or \
                                 1.0
        settings["face_alpha"] = kwargs.get("alpha", None) or \
                                 kwargs.get("face_alpha", None) or \
                                 kwargs.get("fa", None) or \
                                 0.3
        settings["face_color"] = kwargs.get("color", None) or \
                                 kwargs.get("face_color", None) or \
                                 kwargs.get("fc", None) or \
                                 "#000000"

        settings["color"] = kwargs.get("color", None) or \
                            kwargs.get("c", None) or \
                            settings["edge_color"] or \
                            settings["face_color"]

        # Need to replace "plum" with "#DDA0DD" and "c" with "#00FFFF" (cyan).
        for key, color in settings.items():
            if 'color' in key:
                color = self.color_dict.get(color, color)
                color = self.html_color_codes.get(color, color)
                settings[key] = color

        settings["closed"] = kwargs.get("closed", None)

        return settings

    def plot(self, lats, lngs, color=None, c=None, **kwargs):
        color = color or c
        kwargs.setdefault("color", color)
        settings = self._process_kwargs(kwargs)
        path = zip(lats, lngs)
        self.paths.append((path, settings))

    # create the html file which include one google map and all points and
    # paths
    def draw(self, htmlfile, api_key=None):
        f = open(htmlfile, 'w')
        f.write('<html>\n')
        f.write('<head>\n')
        f.write(
            '<meta name="viewport" content="initial-scale=1.0, user-scalable=no" />\n')
        f.write(
            '<meta http-equiv="content-type" content="text/html; charset=UTF-8"/>\n')
        f.write('<title>Google Maps - pygmaps </title>\n')
        f.write('<script type="text/javascript" src="https://maps.googleapis.com/maps/api/js?libraries=visualization&sensor=true_or_false"></script>\n')
        f.write('<script type="text/javascript">\n')
        f.write('\tfunction initialize() {\n')
        self.write_map(f)
        self.write_points(f)
        self.write_text(f)
        f.write('\t}\n')
        f.write('</script>\n')
        f.write('</head>\n')
        f.write(
            '<body style="margin:0px; padding:0px;" onload="initialize()">\n')
        f.write(
            '\t<div id="map_canvas" style="width: 100%; height: 100%;"></div>\n')
        if api_key:
            print 'api key is %s' % api_key
            f.write('<script async defer src="https://maps.googleapis.com/maps/api/js?key=' + api_key + '&callback=initMap"></script>')

        f.write('</body>\n')
        f.write('</html>\n')
        f.close()

    #############################################
    # # # # # # Low level Map Drawing # # # # # #
    #############################################

    def write_points(self, f):
        for point in self.points:
            self.write_point(f, point[0], point[1], point[2], point[3])

    def write_text(self, f):
        for text_point in self.text_points:
            self.write_text_point(f, text_point[0], text_point[1], text_point[2], text_point[3])

    def write_text_point(self, f, lat, lon, color, text):
        f.write('\t\tvar latlng = new google.maps.LatLng(%f, %f);\n' %
                (lat, lon))
        f.write('\t\tvar img = new google.maps.MarkerImage(\'%s\');\n' %
                (self.coloricon % 'clear'))
        f.write('\t\tvar marker = new google.maps.Marker({\n')

        f.write('\t\tlabel: {\
    color: "%(1)s",\
    fontWeight: "bold",\
    text: "%(2)s" },\n' % {'1':color, '2': text})
        f.write('\t\ticon: img,\n')
        f.write('\t\tposition: latlng\n')
        f.write('\t\t});\n')
        f.write('\t\tmarker.setMap(map);\n')
        f.write('\n')

    def get_cycle(self, lat, lng, rad):
        # unit of radius: meter
        cycle = []
        d = (rad / 1000.0) / 6378.8
        lat1 = (math.pi / 180.0) * lat
        lng1 = (math.pi / 180.0) * lng

        r = [x * 10 for x in range(36)]
        for a in r:
            tc = (math.pi / 180.0) * a
            y = math.asin(
                math.sin(lat1) * math.cos(d) + math.cos(lat1) * math.sin(d) * math.cos(tc))
            dlng = math.atan2(math.sin(
                tc) * math.sin(d) * math.cos(lat1), math.cos(d) - math.sin(lat1) * math.sin(y))
            x = ((lng1 - dlng + math.pi) % (2.0 * math.pi)) - math.pi
            cycle.append(
                (float(y * (180.0 / math.pi)), float(x * (180.0 / math.pi))))
        return cycle

    def write_map(self,  f):
        f.write('\t\tvar centerlatlng = new google.maps.LatLng(%f, %f);\n' %
                (self.center[0], self.center[1]))
        f.write('\t\tvar myOptions = {\n')
        f.write('\t\t\tzoom: %d,\n' % (self.zoom))
        f.write('\t\t\tcenter: centerlatlng,\n')
        f.write('\t\t\tmapTypeId: %s, \n' % (self.map_type))
        f.write('\t\t};\n')
        f.write(
            '\t\tvar map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);\n')
        f.write('\n')

    def write_point(self, f, lat, lon, color, title):
        f.write('\t\tvar latlng = new google.maps.LatLng(%f, %f);\n' %
                (lat, lon))
        f.write('\t\tvar img = new google.maps.MarkerImage(\'%s\');\n' %
                (self.coloricon % color))
        f.write('\t\tvar marker = new google.maps.Marker({\n')
        f.write('\t\ttitle: "%s",\n' % title)
        f.write('\t\ticon: img,\n')
        f.write('\t\tposition: latlng\n')
        f.write('\t\t});\n')
        f.write('\t\tmarker.setMap(map);\n')
        f.write('\n')
