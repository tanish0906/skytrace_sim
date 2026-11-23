# SKYTRACE — ROS 2 Airspace Surveillance & Spoofing Detection

3D air-defense radar and live aircraft tracking simulation built with **ROS 2**, **RViz2**, **OpenSky Network ADS-B**, and an integrated **AEROGUARD** web dashboard (`http://localhost:8080`).

## Features
- Live ADS-B aircraft tracking via OpenSky Network API (250 km coverage)
- 3D URDF jet models (`aircraft.urdf` / `base_link.dae`) with 20 Hz dead-reckoning
- 3D military AESA ground radar installation, rotating sweep, range rings, and desert sand terrain
- Real-time GNSS spoofing checks (altitude drift, position jump, missing callsign)
- Browser-based AEROGUARD dashboard with live scope, logs, and spoof/ghost threat injection

## Build & Run
```bash
mkdir -p ~/skytrace_ws/src
cd ~/skytrace_ws/src
git clone https://github.com/<your-username>/skytrace_sim.git
cd ~/skytrace_ws
colcon build --packages-select skytrace_sim
source install/setup.bash
ros2 launch skytrace_sim skytrace_opensky.launch.py
```

Then open `http://localhost:8080` in your browser.
