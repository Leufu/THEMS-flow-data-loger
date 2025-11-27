import pandas as pd
import folium
import branca.colormap as cm

# 1. Cargar y limpiar
df = pd.read_csv('20251127.csv')
df_clean = df[(df['Lat_deg'] != 0) & (df['Lon_deg'] != 0)]

# 2. Crear mapa centrado en el promedio de las coordenadas
center_lat = df_clean['Lat_deg'].mean()
center_lon = df_clean['Lon_deg'].mean()
m = folium.Map(location=[center_lat, center_lon], zoom_start=15)

# 3. Crear escala de colores para la velocidad
colormap = cm.LinearColormap(colors=['blue', 'green', 'yellow', 'red'], 
                             vmin=df_clean['Vel_Knots'].min(), 
                             vmax=df_clean['Vel_Knots'].max(),
                             caption='Velocidad (Nudos)')
m.add_child(colormap)

# 4. Añadir puntos al mapa
for index, row in df_clean.iterrows():
    folium.CircleMarker(
        location=[row['Lat_deg'], row['Lon_deg']],
        radius=3,
        color=colormap(row['Vel_Knots']),
        fill=True,
        fill_opacity=0.7,
        popup=f"Hora: {row['Hora']}<br>Vel: {row['Vel_Knots']} Nud<br>Consumo: {row['Flujo_Lmin']} L/min"
    ).add_to(m)

# 5. Guardar mapa
m.save('mi_ruta_interactiva.html')
print("Mapa guardado como 'mi_ruta_interactiva.html'. Ábrelo en tu navegador.")