import time
import math
import socket
import numpy as np
from pynmeagps import NMEAReader
from requests import post, get, exceptions

from config import *

class KalmanFilter:
    """A simple Kalman filter for smoothing 2D location data."""
    def __init__(self, process_variance, measurement_variance):
        self.initialized = False
        self.dt = 1.0
        self.F = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.eye(4) * process_variance
        self.R = np.eye(2) * measurement_variance
        self.P = np.eye(4) * 1000
        self.x = np.zeros(4)

    def initialize(self, z):
        self.x = np.array([z[0], z[1], 0, 0])
        self.initialized = True
        print(f"Kalman filter initialized at: {self.x[0:2]}")

    def predict(self, dt):
        self.dt = dt
        self.F[0, 2], self.F[1, 3] = self.dt, self.dt
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)
        return self.x[0:2]

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_dynamic_radius(speed_kmh, pdop):
    radius = MIN_RADIUS_METERS
    try:
        pdop = int(float(pdop)) if pdop and str(pdop).strip() else 99
    except (ValueError, TypeError):
        pdop = 99  # Default to worst case PDOP if conversion fails
    if pdop > 6: radius += 200
    elif pdop > 4: radius += 100
    elif pdop > 2: radius += 50
    radius += speed_kmh * 1.5
    return int(min(max(radius, MIN_RADIUS_METERS), MAX_RADIUS_METERS))

def get_previous_state(entity_id):
    """Fetches the last state from Home Assistant."""
    try:
        url = f"{HA_URL}/api/states/{entity_id}"
        response = get(url, headers=HEADERS, timeout=5)
        response.raise_for_status()
        data = response.json()
        timestamp = data["attributes"]["timestamp"]
        # Ensure timestamp is an int (convert from string if needed)
        if isinstance(timestamp, str):
            timestamp = int(timestamp)
        elif isinstance(timestamp, float):
            timestamp = int(timestamp)
        return {
            "lat": float(data["attributes"]["latitude"]),
            "lon": float(data["attributes"]["longitude"]),
            "timestamp": timestamp
        }
    except (exceptions.RequestException, KeyError, ValueError) as e:
        print(f"Could not get previous state for {entity_id}: {e}. Will proceed with first-time update logic.")
        return None

def post_state_update(entity_id, data):
    try:
        url = f"{HA_URL}/api/states/{entity_id}"
        response = post(url, headers=HEADERS, json=data, timeout=10)
        response.raise_for_status()
        print(f"Successfully updated state for {entity_id}.")
        return True
    except exceptions.RequestException as e:
        print(f"Error updating state for {entity_id}: {e}")
        return False

def post_device_tracker_update(data):
    try:
        url = f"{HA_URL}/api/services/device_tracker/see"
        response = post(url, headers=HEADERS, json=data, timeout=10)
        response.raise_for_status()
        print(f"Successfully updated device_tracker.{data.get('dev_id')}.")
        return True
    except exceptions.RequestException as e:
        print(f"Error updating device_tracker.{data.get('dev_id')}: {e}")
        return False

def main():
    kf = KalmanFilter(process_variance=0.01, measurement_variance=10) if 'KALMAN_ENABLED' in globals() and KALMAN_ENABLED else None
    latest_pdop, latest_sat_count, latest_altitude = 99.0, 0, 0.0
    last_filter_time = time.time()
    
    last_check_time = 0
    last_forced_update_time = time.time()

    print("Starting GPS to Home Assistant service...")
    print(f"Kalman filter enabled: {bool('KALMAN_ENABLED' in globals() and KALMAN_ENABLED)}")
    while True:
        try:
            print(f"Connecting to ser2net at {REMOTE_HOST}:{REMOTE_PORT}...")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((REMOTE_HOST, REMOTE_PORT))
                print("Connection successful.")
                
                stream = sock.makefile('rb')
                nmr = NMEAReader(stream)
                
                for _, parsed_data in nmr:
                    if not parsed_data: 
                        continue

                    # Use match case for NMEA message processing
                    match parsed_data.msgID:
                        case 'GGA':
                            if hasattr(parsed_data, 'numSV'): 
                                latest_sat_count = parsed_data.numSV
                            if hasattr(parsed_data, 'alt'): 
                                latest_altitude = parsed_data.alt
                        
                        case 'GSA' if hasattr(parsed_data, 'PDOP'):
                            latest_pdop = parsed_data.PDOP
                        
                        case 'RMC' if hasattr(parsed_data, 'status') and parsed_data.status == 'A':
                            current_time = time.time()
                            
                            # Check if we need to force an update due to MAX_UPDATE_INTERVAL_SECONDS
                            force_update = (current_time - last_forced_update_time) >= MAX_UPDATE_INTERVAL_SECONDS
                            time_since_last_check = current_time - last_check_time
                            
                            # Use match case for update decision logic
                            match (force_update, time_since_last_check < MIN_UPDATE_INTERVAL_SECONDS):
                                case (False, True):
                                    # Not forced and too soon since last check
                                    continue
                                case _:
                                    # Either forced update or enough time has passed
                                    pass
                            
                            last_check_time = current_time

                            raw_coords = np.array([parsed_data.lat, parsed_data.lon])
                            if kf is not None:
                                if not kf.initialized:
                                    kf.initialize(raw_coords)
                                    last_filter_time = current_time
                                    last_forced_update_time = current_time
                                    continue
                            
                                dt = current_time - last_filter_time
                                last_filter_time = current_time
                                kf.predict(dt)
                                smoothed_lat, smoothed_lon = kf.update(raw_coords)
                            else:
                                # Kalman filter disabled; use raw coordinates
                                smoothed_lat, smoothed_lon = raw_coords[0], raw_coords[1]

                            previous_state = get_previous_state(DEVICE_TRACKER_ENTITY_ID)
                            
                            calculated_speed_kmh = 0.0
                            
                            # Process update validation with proper type handling
                            if previous_state and not force_update:
                                # Has previous state and not forced - check distance and speed
                                distance = calculate_distance(previous_state["lat"], previous_state["lon"], smoothed_lat, smoothed_lon)
                                time_delta = current_time - previous_state["timestamp"]
                                calculated_speed_mps = distance / time_delta if time_delta > 0 else 0
                                
                                if distance < MIN_DISTANCE_METERS:
                                    print(f"Skipping update: Moved only {distance:.1f}m.")
                                    continue
                                elif calculated_speed_mps > MAX_SPEED_MPS:
                                    print("Skipping update: Implausible speed detected.")
                                    continue
                                else:
                                    calculated_speed_kmh = calculated_speed_mps * 3.6
                            elif previous_state:
                                # Has previous state and forced update - calculate speed but don't skip
                                distance = calculate_distance(previous_state["lat"], previous_state["lon"], smoothed_lat, smoothed_lon)
                                time_delta = current_time - previous_state["timestamp"]
                                calculated_speed_mps = distance / time_delta if time_delta > 0 else 0
                                calculated_speed_kmh = calculated_speed_mps * 3.6 if calculated_speed_mps <= MAX_SPEED_MPS else parsed_data.spd * 1.852
                            else:
                                # No previous state - use GPS speed
                                calculated_speed_kmh = parsed_data.spd * 1.852

                            dynamic_radius = calculate_dynamic_radius(calculated_speed_kmh, latest_pdop)
                            update_reason = "forced" if force_update else "normal"
                            print(f"Updating HA ({update_reason}): Lat={smoothed_lat:.5f}, Lon={smoothed_lon:.5f}, Speed={calculated_speed_kmh:.1f}km/h, Radius={dynamic_radius}m")

                            zone_data = {
                                "state": "0", "attributes": {
                                    "latitude": smoothed_lat, "longitude": smoothed_lon, "radius": dynamic_radius,
                                    "passive": False, "icon": "mdi:rv-truck", "friendly_name": "RV Location"
                                }
                            }
                            tracker_data = {
                                "dev_id": DEVICE_TRACKER_ENTITY_ID.split('.')[1],
                                "gps": [smoothed_lat, smoothed_lon],
                                "attributes": {
                                    "speed": round(calculated_speed_kmh, 2), "altitude": latest_altitude,
                                    "timestamp": int(current_time), "gps_accuracy": dynamic_radius,
                                    "pdop": latest_pdop, "satellites": latest_sat_count
                                }
                            }

                            post_state_update(ZONE_ENTITY_ID, zone_data)
                            post_device_tracker_update(tracker_data)
                            
                            # Update the forced update timestamp after successful update
                            if force_update:
                                last_forced_update_time = current_time
                        
                        case _:
                            # Handle any other NMEA message types if needed
                            pass

        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            print(f"Network error: {e}. Retrying in 10s...")
            time.sleep(10)
        except KeyboardInterrupt:
            print("Stopping script.")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}. Retrying in 10s...")
            time.sleep(10)

if __name__ == "__main__":
    main()
