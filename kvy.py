from kivy.app import App
from kivy_garden.mapview import MapView, MapMarker

class TestApp(App):
    def build(self):
        mapview = MapView(zoom=15, lat=37.7749, lon=-122.4194)
        marker = MapMarker(lat=37.7749, lon=-122.4194)
        mapview.add_marker(marker)
        return mapview

if __name__ == '__main__':
    TestApp().run()
