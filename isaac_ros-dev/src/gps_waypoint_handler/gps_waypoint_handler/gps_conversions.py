"""Hand-rolled lat/lon → meters utilities.

Retained from the original package for offline / one-shot use; the live
node ``gps_handler_node`` does its own equirectangular linearization in
``latlon_to_local`` (it cannot call ``GPSHandler`` because the live datum
is unknown until the first /gps_fix arrives).

The dead ``current_declination`` parameter present in the original
``apply_heading_offset`` was removed — it was accepted but never
subtracted. (See plan_manifest §1 file manifest, "what gets deleted".)
"""

import math


class GPSHandler:
    def __init__(self, ref_lat: float, ref_long: float, cur_heading: float):
        self.reference_latitude = ref_lat
        self.reference_longitude = ref_long
        # Heading offset (degrees) from world / target frame to the
        # ``apply_heading_offset`` rotation reference. Definition is
        # caller-specific.
        self.current_heading = cur_heading

    def calculate_distance(self, target_latitude: float,
                           target_longitude: float):
        """Equirectangular projection ``(target_lat, target_lon) →
        (x_east_m, y_north_m)`` around the constructor's reference."""
        radius = 6_371_000  # Earth radius (m)
        ref_lat_rads = math.radians(self.reference_latitude)
        ref_long_rads = math.radians(self.reference_longitude)
        target_lat_rads = math.radians(target_latitude)
        target_long_rads = math.radians(target_longitude)
        delta_lat = target_lat_rads - ref_lat_rads
        delta_lon = target_long_rads - ref_long_rads
        y_distance = delta_lat * radius
        x_distance = delta_lon * radius * math.cos(ref_lat_rads)
        return x_distance, y_distance

    def apply_heading_offset(self, x: float, y: float):
        """Rotate ``(x, y)`` by ``self.current_heading`` degrees CCW."""
        cur_heading_rads = math.radians(self.current_heading)
        new_x = x * math.cos(cur_heading_rads) - y * math.sin(cur_heading_rads)
        new_y = x * math.sin(cur_heading_rads) + y * math.cos(cur_heading_rads)
        return new_x, new_y
