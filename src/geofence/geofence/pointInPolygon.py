#%%
# Funktion zum Überprüfen, ob sich die Position im Polygon befindet
def is_inside_polygon(current_position, polygon):
    point = Point(current_position[0], current_position[1])
    return polygon.contains(point)

def switch_lat_long (points):
    switched = [(punkt[1], punkt[0]) for punkt in points]
    return switched

def main():

    # Erstelle eine Karte
    vineyard_coords =  [[9.991449,49.749369],[9.991196,49.749865],[9.991749,49.749903],[9.992441,49.749189],[9.991867,49.748911],[9.991449,49.749369]]
    vineyard_coords = switch_lat_long(vineyard_coords)
    current_position = [49.749560069405454,9.991673846702488] #true
    #current_position = [49.749145067791346, 9.991721831520223] #false
    polygon=Polygon(vineyard_coords)
    inside_polygon = is_inside_polygon(current_position, polygon)

    print("Test:", inside_polygon)


    m = folium.Map(location=current_position, zoom_start=18, tiles=None)
    folium.TileLayer("OpenStreetMap",attr="OpenStreetMap").add_to(m)
    folium.TileLayer("CyclOSM",attr="CyclOSM").add_to(m)
    folium.TileLayer("https://tile.thunderforest.com/outdoors/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                    name="Thunderforest.Outdoors",
                    attr="Thunderforest.Outdoors",max_zoom=22).add_to(m)
    folium.TileLayer("https://tile.thunderforest.com/landscape/{z}/{x}/{y}.png?apikey=da5cdf92f11440f09fbbed466d51c86b",
                    name="Thunderforest.Landscape",
                    attr="Thunderforest.Landscape",max_zoom=22).add_to(m)
    folium.TileLayer("Esri.WorldImagery",attr="Esri.WorldImagery",max_zoom=19).add_to(m) 


    folium.Marker(
        location=current_position,
        popup=folium.Popup(u"Ça c'est chouette", parse_html=True, max_width="100%"),
    ).add_to(m)

    folium.Polygon(
        locations=vineyard_coords,
        color="blue",
        weight=6,
        fill_color="red",
        fill_opacity=0.5,
        fill=True,
        popup="Test mopint",
        tooltip="Click me!",
    ).add_to(m)

    display(m)





if __name__ == '__main__':
    import folium
    import time
    from shapely.geometry import Point, Polygon
    from IPython.display import display, HTML
    main()

    # test_coords =  [[9.991449,49.749369],[9.991196,49.749865],[9.991749,49.749903],[9.992441,49.749189],[9.991867,49.748911],[9.991449,49.749369]]

    # current_position = [ 9.991673846702488,49.749560069405454,] 
    # polygon_T = Polygon(test_coords)
    # inside_polygon = is_inside_polygon(current_position, polygon_T)
    # print("Test:", inside_polygon)
    # make_polygon(test_coords)