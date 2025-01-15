from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
# from kivy.uix.mapview import MapView, MapMarker
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.textinput import TextInput
from kivy.clock import Clock
from kivy.garden.mapview import MapView, MapMarker
from kivy.garden import mapview

import random


class GuidanceRobotUI(App):
    def build(self):
        # Main layout
        main_layout = BoxLayout(orientation='vertical')

        # Header
        header = Label(text="Guidance Robot Interface", font_size=30, size_hint=(1, 0.1))
        main_layout.add_widget(header)

        # Map View Section
        self.map_view = MapView(zoom=15, lat=37.7749, lon=-122.4194)  # Sample coordinates (San Francisco)
        self.robot_marker = MapMarker(lat=37.7749, lon=-122.4194)
        self.map_view.add_marker(self.robot_marker)
        main_layout.add_widget(self.map_view)

        # Input Section
        input_layout = GridLayout(cols=2, size_hint=(1, 0.2))
        
        input_layout.add_widget(Label(text="Enter Destination:", font_size=20))
        self.destination_input = TextInput(hint_text="e.g., Room A", multiline=False)
        input_layout.add_widget(self.destination_input)

        # Add Buttons
        self.start_button = Button(text="Start Navigation", font_size=20)
        self.start_button.bind(on_press=self.start_navigation)
        input_layout.add_widget(self.start_button)

        self.stop_button = Button(text="Stop Navigation", font_size=20)
        self.stop_button.bind(on_press=self.stop_navigation)
        input_layout.add_widget(self.stop_button)

        main_layout.add_widget(input_layout)

        # Status Section
        self.status_label = Label(text="Status: Waiting for input...", font_size=18, size_hint=(1, 0.1))
        main_layout.add_widget(self.status_label)

        return main_layout

    def start_navigation(self, instance):
        destination = self.destination_input.text
        if destination:
            self.status_label.text = f"Navigating to {destination}..."
            # Simulate robot movement by updating position on the map
            Clock.schedule_interval(self.update_robot_position, 1)
        else:
            self.status_label.text = "Error: Please enter a destination."

    def stop_navigation(self, instance):
        self.status_label.text = "Navigation stopped."
        Clock.unschedule(self.update_robot_position)

    def update_robot_position(self, dt):
        # Simulate robot movement
        new_lat = self.robot_marker.lat + random.uniform(-0.001, 0.001)
        new_lon = self.robot_marker.lon + random.uniform(-0.001, 0.001)
        
        self.robot_marker.lat = new_lat
        self.robot_marker.lon = new_lon
        self.map_view.center_on(new_lat, new_lon)

if __name__ == "__main__":
    GuidanceRobotUI().run()
