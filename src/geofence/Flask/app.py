from flask import Flask, render_template, request, jsonify
import os
import json

app = Flask(__name__)

# Pfad zum Speichern der GeoJSON-Dateien
UPLOAD_FOLDER = 'geojson_files'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/confirm', methods=['POST'])
def confirm():
    selected_file = request.form['selected_file']
    # Hier könnten Sie weitere Aktionen wie das Lesen der GeoJSON-Datei und das Plotten der Koordinaten durchführen
    return jsonify({'message': 'Geofence festgelegt'})

@app.route('/save_geojson', methods=['POST'])
def save_geojson():
    data = request.json
    filename = data['filename']
    geojson = data['geojson']
    with open(os.path.join(app.config['UPLOAD_FOLDER'], filename), 'w') as f:
        json.dump(geojson, f)
    return jsonify({'message': 'GeoJSON gespeichert'})

@app.route('/get_geojson_files')
def get_geojson_files():
    files = os.listdir(app.config['UPLOAD_FOLDER'])
    return jsonify({'files': files})

if __name__ == '__main__':
    app.run(debug=True)
