package dji.v5.ux.mapkit.maplibre.map

/**
 * @author feel.feng
 * @time 2024/07/10 17:27
 * @description: 升级9.5.x后 style 被移除
 */
object MaplibreStyle {
    const val MAPBOX_STREETS = "https://basemaps.cartocdn.com/gl/voyager-gl-style/style.json"
    const val SATELLITE_STREETS = "https://basemaps.cartocdn.com/gl/positron-gl-style/style.json"
    const val SATELLITE = """{
      "version": 8,
      "sources": {
        "esri-satellite": {
          "type": "raster",
          "tiles": [
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          ],
          "tileSize": 256,
          "attribution": "Tiles &copy; Esri"
        }
      },
      "layers": [
        {
          "id": "esri-satellite-layer",
          "type": "raster",
          "source": "esri-satellite",
          "minzoom": 0,
          "maxzoom": 19
        }
      ]
    }"""
}