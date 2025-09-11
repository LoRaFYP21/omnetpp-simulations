# FLoRa

FLoRa (Framework for LoRa) is a simulation framework designed for end-to-end simulations of LoRa networks. Built on the [OMNeT++](https://omnetpp.org/) network simulator, it also integrates components from the [INET framework](https://inet.omnetpp.org/).

The framework allows users to create LoRa networks with modules for LoRa nodes, gateway(s), and a network server. Application logic can be deployed as independent modules connected with the network server. Additionally, FLoRa supports dynamic configuration management through Adaptive Data Rate (ADR), and energy consumption statistics are collected for each node in the network.

For more details, visit the official website: [flora.aalto.fi](http://flora.aalto.fi/).

## Features

- **LoRa network simulation**: Includes nodes, gateways, and network server modules.
- **Adaptive Data Rate (ADR)**: Dynamic configuration of nodes.
- **Energy consumption tracking**: Monitors energy usage in nodes.
- **Pre-configured scenarios**: Various simulation scenarios for testing different network conditions.

## Prerequisites - Recommonded

- OMNeT++ 5.3   
  [Download OMNeT++](https://omnetpp.org/)
  
- INET framework 3.7.1 
  [Download INET](https://inet.omnetpp.org/)

## Installation

1. **Install OMNeT++:**
   - Download and install OMNeT++ 5.3 (or later) from the [OMNeT++ website](https://omnetpp.org/).
   
2. **Install the INET framework:**
   - Download and install INET framework version 3.7.1 (or later) from the [INET website](https://inet.omnetpp.org/). Ensure OMNeT++ is installed as a prerequisite.

3. **Import the Project in OMNeT++:**
   - Open OMNeT++, go to `File > Import > Git > Projects from Git`.
   - Enter the repository URL: `https://github.com/LoRaFYP19/omnetpp-mesh-tester.git`.
   - Complete the import to load the project into OMNeT++.

## How to Deploy and Run Simulations

### Step 1: Open the Simulations Folder
- Navigate to the **Project Explorer** panel in OMNeT++.
- Inside the project structure, locate the `simulations` folder where pre-configured `.ini` files (simulation scenarios) are stored.

### Step 2: Select a Simulation Scenario
- Each `.ini` file in the `simulations` folder corresponds to a specific test setup and LoRa network configuration.
- Choose a `.ini` file based on the scenario you want to simulate.

### Step 3: Run the Simulation
- Once a `.ini` file is selected, click on the green play button in the horizontal menu bar at the top of OMNeT++.
- The simulation may take some time to load and will display in a new window.

### Step 4: View and Monitor the Network
- The new window will show a visual representation of the LoRa network.
- Red lines represent packets being transmitted across the network.
- You can observe the packet flow and other network behaviors during the simulation.

### Step 5: Adjust and Analyze
- During or after the simulation, you can adjust settings, analyze results, pause, or reset the simulation as needed.

## Contributing

We welcome contributions to improve FLoRa. Feel free to submit issues or pull requests.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
