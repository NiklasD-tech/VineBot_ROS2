
#!/usr/bin/env python


#Folium Doku https://python-visualization.github.io/folium/latest/index.html
#karten: http://leaflet-extras.github.io/leaflet-providers/preview/index.html


#api key THunderforest : https://manage.thunderforest.com/users/sign_up?plan=hobby-project APIKEY Umsonst!!!!
# OpenCycleMap
#     https://tile.thunderforest.com/cycle/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Transport
#     https://tile.thunderforest.com/transport/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Landscape
#     https://tile.thunderforest.com/landscape/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Outdoors
#     https://tile.thunderforest.com/outdoors/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Transport Dark
#     https://tile.thunderforest.com/transport-dark/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Spinal Map
#     https://tile.thunderforest.com/spinal-map/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Pioneer
#     https://tile.thunderforest.com/pioneer/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Mobile Atlas
#     https://tile.thunderforest.com/mobile-atlas/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Neighbourhood
#     https://tile.thunderforest.com/neighbourhood/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 
# Atlas
#     https://tile.thunderforest.com/atlas/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b 


#%%



def read_geojson(filename):
    with open(filename, 'r') as f:
        geojson_data = json.load(f)
        points = []
        for feature in geojson_data['features']:
            if 'geometry' in feature and feature['geometry']['type'] == 'Polygon':
                for coordinate in feature['geometry']['coordinates'][0]:
                    points.append(coordinate)
        return points


def on_dropdown_change(change):
    actuellVineyard=[]
    selected_file = change['new']
    if selected_file:
        points = read_geojson(selected_file)
        print("Points in", selected_file, ":", points)
    return points


# Funktion zum Überprüfen, ob sich die Position im Polygon befindet
def is_inside_polygon(current_position, polygon):
    point = Point(current_position[0], current_position[1])
    return polygon.contains(point)

def switch_lat_long (points):
    switched = [(punkt[1], punkt[0]) for punkt in points]
    return switched



if __name__ == '__main__':

    import folium
    from folium.plugins import Draw, scroll_zoom_toggler
    from IPython.display import display, HTML
    import json
    import ipywidgets as widgets
    import os
    from shapely.geometry import Point, Polygon


    # Variable zur Auswahl des Kartentyps (z.B. "OpenStreetMap", "Stamen Terrain", "Stamen Toner", usw.)
    #map_type = "OpenStreetMap"
    #map_type = "Esri.WorldImagery" #bestes image of vineyards
    #map_type = "CyclOSM"  #langsam aber sehr detailiert,bester zoom

    start_coordinates = [49.74908110125702, 9.991184439987288]
    m = folium.Map(location=start_coordinates, zoom_start=18, tiles=None)
    #folium.plugins.ScrollZoomToggler().add_to(m)

    # Hinzufügen des Zeichenwerkzeugs zur Karte
    draw = Draw(export=True, draw_options={'circlemarker':False,'polyline': False, 'polygon': True, 'rectangle': False, 'circle': False, 'marker': False}).add_to(m)

    # Hizufügen der kartenauswahl
    folium.TileLayer("OpenStreetMap",attr="OpenStreetMap").add_to(m)
    folium.TileLayer("CyclOSM",attr="CyclOSM").add_to(m)
    folium.TileLayer("https://tile.thunderforest.com/outdoors/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                    name="Thunderforest.Outdoors",
                    attr="Thunderforest.Outdoors",max_zoom=22).add_to(m)
    folium.TileLayer("https://tile.thunderforest.com/landscape/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                    name="Thunderforest.Landscape",
                    attr="Thunderforest.Landscape",max_zoom=22).add_to(m)
    folium.TileLayer("Esri.WorldImagery",attr="Esri.WorldImagery",max_zoom=19).add_to(m) 


    # Hizufügen einer Geojason / Weinberg
    borderStyle={
        "color": "red",
        "weight": 1,
        "fillColor": "red",
        "fillOpacity": 0.2
    }
# Ordnerpfad, in dem sich die GeoJSON-Dateien befinden
    folder_path = "/home/irobot/ros2_ws/src/geofence/geofence"

    geojsonNames=[]
    wineyardenames=[]
    for filename in os.listdir(folder_path):
        if filename.endswith(".geojson"):
            folium.GeoJson(os.path.join(folder_path, filename),
                name=filename,  # Entferne die ".geojson" -Erweiterung aus dem Namen
                style_function=lambda x: borderStyle).add_to(m)
            geojsonNames.append(filename)
            wineyardenames.append(filename[:-8])
            

                
    # Add the full screen button.                                               
    folium.plugins.Fullscreen(
        position="bottomright",
        title="Expand me",
        title_cancel="Exit me",
        force_separate_button=True,
    ).add_to(m)

    # Anzeige der Karte
    folium.LayerControl().add_to(m)
    
    m.save("map.html")
    #m.show_in_browser()

    geojsonNames.sort()
    wineyardenames.sort()
    dropdown_options = ''.join(f'<option value="{filename}">{filename}</option>' for filename in geojsonNames)
   
    dropdown = widgets.Dropdown(options=geojsonNames, description='Choose GeoJSON file:')
    dropdown.observe(on_dropdown_change, names='value')
    

    display(dropdown)
   

    # Erstelle eine Karte
    vineyard_coords =  [[9.991449,49.749369],[9.991196,49.749865],[9.991749,49.749903],[9.992441,49.749189],[9.991867,49.748911],[9.991449,49.749369]]
    vineyard_coords = switch_lat_long(vineyard_coords)
    #current_position = [49.749560069405454,9.991673846702488] #true
    current_position = [49.749145067791346, 9.991721831520223] #false
    polygon=Polygon(vineyard_coords)
    inside_polygon = is_inside_polygon(current_position, polygon)

    print("Test:", inside_polygon)

    folium.Marker(
        location=current_position,
        popup=folium.Popup(u"Ça c'est chouette", parse_html=True, max_width="100%"),
    ).add_to(m)

    folium.Polygon(
        locations=vineyard_coords,
        color="red",
        weight=3,
        fill_color="green",
        fill_opacity=0.5,
        fill=True,
        popup="Test mopint",
        tooltip="Click me!",
    ).add_to(m)


display(m)
## Hinzufügen der Schaltfläche zum Bestätigen und Schließen
# confirm_button_html = """
# <div style="position: fixed; top: 10px; right: 250px; z-index: 1000;">
#     <button onclick="handleButtonClick()">Bestätigen und Schließen</button>
# </div>
# <script>
#     function handleButtonClick() {
#         var kernel = Jupyter.notebook.kernel;
#         kernel.execute("on_button_click()");
#     }
# </script>
# """
# m.get_root().html.add_child(folium.Element(confirm_button_html))

#%%



