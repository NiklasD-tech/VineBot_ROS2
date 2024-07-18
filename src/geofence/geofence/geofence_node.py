import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavSTATUS
from shapely.geometry import Point, Polygon
import geojson
import folium
import math
import numpy as np
import os
import time
from folium.plugins import Draw, scroll_zoom_toggler

class GPSSubscriber(Node):

    def __init__(self):
        super().__init__('GPS_subscriber')
        self.folder_path = "/home/irobot/ros2_ws/src/geofence/geofence/Vineyards"
        self.vineyard_polygons = self.load_all_polygons(self.folder_path)

        self.fix_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.listener_callback,
            10)
        self.navstatus_subscription = self.create_subscription(
            NavSTATUS,
            '/navstatus',
            self.navstatus_listener_callback,
            10)

        self.latitude = None
        self.longitude = None
        self.current_position = [self.latitude, self.longitude]
        self.lastPositions = []
        self.allPositions = []
        self.inside_polygon = False
        self.cnt = 0
        self.gps_signal = 0
        self.gps_fix = 0
        self.closest_vineyard = None
        self.closest_vineyard_name = None
        self.shortest_distance_to_vineyard=99999999999
        self.initialization_start_time = time.time()
        self.initialization_duration = 10  # Dauer der Initialisierung in Sekunden
        self.timer = self.create_timer(1.0, self.update_map)

    def load_all_polygons(self, folder_path):
        vineyard_polygons = []
        for filename in os.listdir(folder_path):
            if filename.endswith(".geojson"):
                polygon = self.load_polygon_from_geojson(os.path.join(folder_path, filename))
                vineyard_polygons.append((filename, polygon))
        return vineyard_polygons
    
    
    def shortest_distance_to_polygon(self, polygon):
        if self.latitude is not None and self.longitude is not None:
            inside_polygon = self.is_inside_polygon(self.current_position, polygon)
            if inside_polygon:
                distances_haversine = self.calculate_all_distances_to_boundary_haversine(self.current_position, polygon)
                shortest_distance = min(distances_haversine)
            else:
                distances_haversine = self.calculate_all_distances_to_boundary_haversine(self.current_position, polygon)
                shortest_distance = min(distances_haversine)
        return shortest_distance
    
    
    def find_closest_vineyard(self, current_position):
        closest_vineyard = None
        closest_distance = float('inf')
        closest_vineyard_name = None

        for name, polygon in self.vineyard_polygons:
            distance = self.shortest_distance_to_polygon(polygon)
            if distance < closest_distance:
                closest_distance = distance
                closest_vineyard = polygon
                closest_vineyard_name = name
                closest_vineyard_name = closest_vineyard_name[:-8]


        return closest_vineyard, (closest_vineyard_name, closest_distance)


    def listener_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.get_logger().info(f'Coords: {msg.latitude}, {msg.longitude}')
        if self.latitude is not None and self.longitude is not None:
            self.current_position = [self.latitude, self.longitude]
            self.lastPositions.append(self.current_position)
            if len(self.lastPositions) > 50:
                self.lastPositions.pop(0)
            self.allPositions.append(self.current_position)
            if len(self.allPositions) > 5000:
                self.allPositions.pop(0)

            if time.time() - self.initialization_start_time > self.initialization_duration:
                closest_vineyard, distance_to_closest_vineyard = self.find_closest_vineyard(self.current_position)
                if self.closest_vineyard != closest_vineyard:
                    self.closest_vineyard = closest_vineyard
                    self.closest_vineyard_name, self.closest_distance = distance_to_closest_vineyard
                    self.get_logger().info(f'Nächster Weinberg: {self.closest_vineyard_name}, Entfernung: {self.closest_distance:.2f} Meter')

                if self.closest_vineyard is not None:
                    self.inside_polygon = self.is_inside_polygon(self.current_position, self.closest_vineyard)
                    if self.inside_polygon:
                        distances_haversine = self.calculate_all_distances_to_boundary_haversine(self.current_position, self.closest_vineyard)
                        self.get_logger().info(f'Innerhalb des Weinbergs')
                        for idx, distance in enumerate(distances_haversine):
                            self.get_logger().info(f'Entfernung zur Begrenzung {idx}: {distance:.2f} Meter')
                        self.shortest_distance_to_vineyard=min(distances_haversine)
                    else:
                        distances_haversine = self.calculate_all_distances_to_boundary_haversine(self.current_position, self.closest_vineyard)
                        self.get_logger().info('AUßERHALB des Weinbergs')
                        self.shortest_distance_to_vineyard=min(distances_haversine)
                        self.get_logger().info(f'Entfernung zum Weinberg {self.closest_vineyard_name}: {self.shortest_distance_to_vineyard:.2f} Meter')

    def navstatus_listener_callback(self, msg):
        self.gps_signal = msg.flags2
        self.gps_fix = msg.gps_fix
        self.get_logger().info(
            f'\n'
            f'i_tow: {msg.i_tow}\n'
            f'gps_fix: {msg.gps_fix}\n'
            f'flags: {msg.flags}\n'
            f'fix_stat: {msg.fix_stat}\n'
            f'flags2: {msg.flags2}\n'
            f'ttff: {msg.ttff}\n'
            f'msss: {msg.msss}\n'
        )
        if msg.gps_fix != 3:
            self.get_logger().info(f'ACHTUNG: kein 3D-Fix')
            f'---------------------------------'
            f'\n'
        else:
            f'---------------------------------'
            f'\n'

    
    def is_inside_polygon(self, current_position, polygon):
        point = Point(current_position[1], current_position[0])
        return polygon.contains(point)
    
    def load_polygon_from_geojson(self, file_path):
        with open(file_path) as f:
            geojson_data = geojson.load(f)
        coordinates = geojson_data['features'][0]['geometry']['coordinates'][0]
        polygon = Polygon(coordinates)
        return polygon

    def calculate_all_distances_to_boundary_haversine(self, position, polygon):
        distances = []

        for i in range(len(polygon.exterior.coords) - 1):
            p1 = polygon.exterior.coords[i]
            p2 = polygon.exterior.coords[i + 1]
            distance = self.point_to_line_haversine_distance(position, [p1[1], p1[0]], [p2[1], p2[0]])
            distances.append(distance)
        return distances

    def haversine(self, coord1, coord2):
        R = 6371000  # Erdradius in Metern
        lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
        lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    def point_to_line_haversine_distance(self, point, line_start, line_end):
        line_vec = np.array([line_end[0] - line_start[0], line_end[1] - line_start[1]])
        point_vec = np.array([point[0] - line_start[0], point[1] - line_start[1]])
        line_len = np.dot(line_vec, line_vec)
        t = max(0, min(1, np.dot(point_vec, line_vec) / line_len))
        projection = line_start + t * line_vec
        return self.haversine(point, projection)

    def add_polyline(self, map_obj, points, c, w):
        polyline = folium.PolyLine(locations=points, color=c, weight=w)
        map_obj.add_child(polyline)

    def clear_map(self, map_obj):
        map_obj._children = {}

    def get_gps_status(self):
        gps_status = "unknown"
        if self.gps_fix==3:
            if self.gps_signal == 8:
                gps_status = "bad"
            elif self.gps_signal == 72:
                gps_status = "Float"
            elif self.gps_signal == 136:
                gps_status = "Fix"
            else:
                gps_status = "unknown"
        else:
            gps_status = "No Signal"
        return(gps_status)
    
    def add_tile_layers(self):
        folium.TileLayer("OpenStreetMap", attr="OpenStreetMap").add_to(self.m)
        folium.TileLayer("CyclOSM", attr="CyclOSM").add_to(self.m)
        folium.TileLayer("https://tile.thunderforest.com/outdoors/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                         name="Thunderforest.Outdoors",
                         attr="Thunderforest.Outdoors", max_zoom=22).add_to(self.m)
        folium.TileLayer("https://tile.thunderforest.com/landscape/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                         name="Thunderforest.Landscape",
                         attr="Thunderforest.Landscape", max_zoom=22).add_to(self.m)
        folium.TileLayer("Esri.WorldImagery", attr="Esri.WorldImagery", max_zoom=19).add_to(self.m)
        

    def add_vineyard_polygons(self):
        border_style = {
            "color": "blue",
            "weight": 1,
            "fillColor": "blue",
            "fillOpacity": 0.2
        }
        for name, polygon in self.vineyard_polygons:
            folium.GeoJson(polygon, name=name, style_function=lambda x: border_style).add_to(self.m)

    def create_map(self):
        self.m = folium.Map(location=self.current_position, zoom_start=21, tiles=None)
        draw = Draw(export=True, draw_options={'circlemarker':False, 'polyline': False, 'polygon': True, 'rectangle': False, 'circle': False, 'marker': False}).add_to(self.m)
        folium.plugins.Fullscreen(
            position="bottomright",
            title="Expand me",
            title_cancel="Exit me",
            force_separate_button=True,
            ).add_to(self.m)
        self.add_tile_layers()
        
        self.add_vineyard_polygons()
        self.add_current_position_marker()
        self.add_legend()
        self.add_polyline(self.m, self.allPositions, "black", 3)
        self.add_polyline(self.m, self.lastPositions, "red", 1)
        folium.LayerControl().add_to(self.m)

    def get_marker_settings(self):
        if self.inside_polygon:
            tooltip_text = 'Innerhalb des Weinbergs'
            inner_color = 'green'
        else:
            tooltip_text = 'Außerhalb des Weinbergs'
            inner_color = 'red'     
        return(inner_color,tooltip_text)

    def add_current_position_marker(self):
        inner_color,tooltip_text=self.get_marker_settings()
        folium.Marker(self.current_position, tooltip=tooltip_text, icon=folium.Icon(color=inner_color)).add_to(self.m)
        return inner_color

    def add_legend(self):
        shortest_distance = self.shortest_distance_to_vineyard
        gps_status=self.get_gps_status()
        text="Kürzester Abstand zum Weinberg " + self.closest_vineyard_name
        inner_color,tooltip_text=self.get_marker_settings()
        inside_text_inside="Innerhalb des Weinbergs: " + self.closest_vineyard_name
        inside_text2_outside="Außerhalb des Weinbergs: " + self.closest_vineyard_name

        legend_html = '''
                    <div style="position: fixed; 
                                bottom: 50px; right: 50px; width: 500px; height: 150px; 
                                border: 2px solid grey; z-index: 9999; font-size: 14px;
                                background-color: white; padding: 10px;">
                        <p style="margin: 0 0 5px 0; font-weight: bold;">Info</p>
                        <div style="display: flex; align-items: center; margin-bottom: 5px;">
                            <svg height="20" width="20">
                                <circle cx="10" cy="10" r="8" stroke="black" stroke-width="1" fill="{inner_color}" />
                            </svg>
                            <span style="margin-left: 5px;">{inside_text}</span>
                        </div>
                        <div style="display: flex; align-items: center;">
                            <p style="margin: 0; font-weight: bold;">{inside_text2}:</p>
                            <span style="margin-left: 10px; color: blue;">{shortest_distance:.2f} Meter</span>
                        </div>
                        <div style="display: flex; align-items: center;">
                            <p style="margin: 0; font-weight: bold;">GPS-Signal:</p>
                            <span style="margin-left: 10px; color: blue;">{gps_status}</span>
                        </div>
                    </div>
                    '''.format(inner_color=inner_color, 
                            inside_text=inside_text_inside if self.inside_polygon else inside_text2_outside,
                            shortest_distance=shortest_distance if shortest_distance is not None else "-",
                            inside_text2="Kürzester Abstand zur Begrenzung" if self.inside_polygon else text,
                            gps_status=gps_status,
                            weinbergsname=self.closest_vineyard_name)

        self.m.get_root().html.add_child(folium.Element(legend_html))

    def update_map(self):
        if self.closest_vineyard is not None:
            if self.latitude is not None and self.longitude is not None and self.closest_vineyard is not None:
                self.create_map()
                self.m.save('/home/irobot/ros2_ws/src/geofence/geofence/subscrMap.html')
                #self.get_logger().info('Map updated and saved as subscrMap.html.')
            else:
                self.get_logger().info('Latitude or Longitude is None, cannot update map.')
        else:
            initialisationTime=round(self.initialization_duration-(time.time() - self.initialization_start_time))
            output= "ACHTUNG INITIALISIERUNG noch : " + str(initialisationTime) + " Sekunden"
            self.get_logger().info(output)

def main(args=None):
    rclpy.init(args=args)
    gps_subscriber = GPSSubscriber()
    rclpy.spin(gps_subscriber)
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
