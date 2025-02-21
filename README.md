# SUAVPy

This project simulates UAV sensing, by integrating SUMO (Simulation of Urban MObility) traffic simulator.

## Requirements

- Python 3.7+ 
- SUMO Eclipse
- requirements.txt

## Configuration

We can run the plugin using uav_gui_run.bat file in which a user interface for configuration or uav_run.bat file for a straightforward version. The simulation is configured using the GUI in uavpy_gui.py or in `config.json` file. 
Below is an example configuration:

```json
{
    "Movement": "Continuous",
    "Remote Server": false,
    "Local GUI": false,
    "Battery Mode":     true,                   // true to enable battery mode, false to disable
    "Battery life(s)":  420,                    // Battery life in seconds (used if Uav Model is "Manual")
    "fov_degrees":      [68, 40],               // Field of view degrees (used if Uav Model is "Manual")
    "uav_speed":        15,                     // UAV speed in m/s (used if Uav Model is "Manual")
    "yaw_speed":        5,                      // UAV yaw speed in degrees/s
    "Gui Option":       true,                   // true to enable GUI, false to disable
    "Uav Mode":         "Hovering",             // Options: "Hovering", "Spinning", "Sampling"
    "Network file":     "NetworkFiles/
                        <filename>.net.xml",    // Path to network file
    "Sumocfg file":     "NetworkFiles/
                        <filename>.sumocfg",    // Path to SUMO configuration file
    "Step length(s)":   1,                      // Simulation step length in seconds
    "Total time(s)":    1000,                   // Total simulation time in seconds
    "Number of UAVs":   2,                      // Number of UAVs in the simulation
    "uav_data":                                 // "Uav_Id": ["time-point","uav_x", "uav_y","uav_z","yaw_angle"]
    {                                           // Keep id's order as it is. 
        "0": [
            [0, 1025, 1589, 0, 0], 
            [10, 1150, 1385, 300, 0],
            [100, 1150, 1585, 300, 0],
            [200, 750, 1585, 300, 0],
            [1080, 1025, 1589, 0, 0]
        ],
        // Additional UAV data...
    }
}
```

## Usage

1. Ensure SUMO is installed and properly configured.
2. Install the required Python packages:
    ```bash
    pip install -r requirements.txt
    ```
3. Directory Structure:
    ```
    /Simulation_folder
    ├── config.json              # Configuration file for the simulation
    ├── main.py                  # Main script to run the simulation
    ├── utils.py                 # Utility functions and classes
    ├── NetworkFiles
    |   ├── network_file.xml     # Network file used by SUMO to define the simulation environment
    |   └── sumo_config.sumocfg  # SUMO configuration file specifying the simulation setup         
    ├── Outputs                  # Directory for output files
    │   ├── uav_output.csv       # CSV file for UAV output data
    |   └── external outputs     # sumo output data   
    ├── images                   # Directory for image files
    │   ├── manual.png           # Icon for manual UAV
    │   ├── mini3pro.png         # Icon for Mini 3 Pro UAV
    │   └── mavic2e.png          # Icon for Mavic 2E UAV
    └── README.md                # This readme file

    ```
4. Prepare your `config.json` file with the desired configuration.
5. Run the gui:
    ```bash
    python uavpy_gui.py
    ```


