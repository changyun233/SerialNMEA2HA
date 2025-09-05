# Serial NMEA to Home Assistant Tracker

A lightweight docker service that reads NMEA GPS data from a TCP serial connection (like ser2net running on OpenWRT) and sends the location data to Home Assistant via its REST API. Perfect for RV and boat owners who want to track their vehicles in Home Assistant.

## Features

- Connects to a TCP server (like ser2net) to read NMEA GPS data
- Parses NMEA sentences to extract location data
- Sends location updates to Home Assistant's REST API
- Creates/updates a device tracker entity in Home Assistant
- Configurable update intervals and distance thresholds
- Runs in a Docker container for easy deployment

## Prerequisites

- A GPS device (tested with RM520N-GL 5G dongle)
- ser2net or similar TCP-to-serial bridge running on your OpenWRT router
- Home Assistant instance with a long-lived access token
- Docker compose installed on your server

## Setup

### 1. Configure ser2net on OpenWRT

On your OpenWRT router, configure ser2net to expose the GPS serial port over TCP. Luci-app-ser2net could be helpful. 

### 2. Configure the Service

1. Copy the example config file:

   ```bash
   cp config.py.example config.py
   ```

2. Edit `config.py` with your settings. Here are the available parameters:

   - **ser2net Connection Settings**
     - `REMOTE_HOST`: IP address of your OpenWRT router running ser2net
     - `REMOTE_PORT`: TCP port where ser2net is exposing the GPS data (default: 5000)

   - **Home Assistant Integration**
     - `HA_URL`: Full URL of your Home Assistant instance (e.g., `http://homeassistant.local:8123`)
     - `TOKEN`: Long-lived access token from Home Assistant
     - `ZONE_ENTITY_ID`: Entity ID for the zone (e.g., `zone.rv`)
     - `DEVICE_TRACKER_ENTITY_ID`: Entity ID for the device tracker (e.g., `device_tracker.rv_gps`), since zone entities do not support timestamp and gps info, we do need to use a device tracker entity

   - **Update Behavior**
     - `MIN_UPDATE_INTERVAL_SECONDS`: Minimum time between updates (default: 10 seconds)
     - `MAX_UPDATE_INTERVAL_SECONDS`: Force update after this time (default: 300 seconds)
     - `MIN_DISTANCE_METERS`: Minimum distance change to trigger an update (default: 20 meters)
     - `MAX_SPEED_MPS`: Filter out unrealistic speed values (default: 1000 m/s)
     - `MIN_RADIUS_METERS`: Minimum radius for the geofence (default: 100 meters)
     - `MAX_RADIUS_METERS`: Maximum radius for the geofence (default: 500 meters)

3. Build and Run with Docker

    ```bash
    docker-compose up --build -d
    ```

### 3. Home Assistant Configuration
The service will automatically create a device tracker entity in Home Assistant. You can add it to your dashboard using a map card:

```yaml
type: map
entities:
  - entity: device_tracker.rv_gps
```

### Troubleshooting
1. No GPS :
- use serial communication app to test the ser2net connection, [coolterm](https://freeware.the-meiers.org/) is a good option
- Ensure your GPS device has a clear view of the sky
- Check if the GPS module is properly connected to your OpenWRT router

### License
MIT

### Contributing
Feel free to submit issues and enhancement requests. Contributions are welcome!