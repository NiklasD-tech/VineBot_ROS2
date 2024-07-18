#%%
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
import folium
from folium.plugins import Draw, scroll_zoom_toggler
from IPython.display import display


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
    "fillOpacity": 0.1
}

folium.GeoJson("Vineyard1.geojson", 
               name="Vineyard1",
               style_function=lambda x:borderStyle).add_to(m)
               
# Add the full screen button.                                               
folium.plugins.Fullscreen(
    position="bottomright",
    title="Expand me",
    title_cancel="Exit me",
    force_separate_button=True,
).add_to(m)

# Hinzufügen der Schaltfläche zum Bestätigen und Schließen
confirm_button_html = """
<div style="position: fixed; top: 10px; right: 250px; z-index: 1000;">
    <button onclick="handleButtonClick()">Bestätigen und Schließen</button>
</div>
<script>
    function handleButtonClick() {
        var kernel = Jupyter.notebook.kernel;
        kernel.execute("on_button_click()");
    }
</script>
"""
m.get_root().html.add_child(folium.Element(confirm_button_html))

# Funktion, die aufgerufen wird, wenn der Button geklickt wird
def on_button_click():
    # Deaktiviere die Anzeige der Karte
    print("close")


# Anzeige der Karte
folium.LayerControl().add_to(m)
display(m)
m.save("map.html")
m.show_in_browser()