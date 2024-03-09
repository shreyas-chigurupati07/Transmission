# Transmission - RRT motion plaaner

## Project Overview

This project involves the implementation of an RRT-based motion planner to extract the mainshaft from a transmission case without causing any collisions, utilizing the WeBots simulation environment for execution and visualization.

## Problem Statement

Design a motion planner that guides the mainshaft of a transmission out of its case without contacting any other components. The simulation requires careful path planning, collision detection, and an understanding of the mechanical constraints within the transmission system.

## Implementation Details

- **Simulation Environment:** WeBots R2023a, chosen for its collision detection capabilities with STL files.
- **Planning Algorithm:** RRT-based planner focusing on collision-free pathfinding in a complex 3D environment.
- **Collision Detection:** Utilizes WeBots' `solidnode.getContactPoints()` function to validate collision-free states during planning.
- **Visualization:** Matplotlib is used for visualization of the network and path due to WeBots' limited plotting capabilities.

## Running the Simulation

1. Install WeBots R2023a from https://cyberbotics.com.
2. Unzip the source code folder and navigate to `Transmission > WeBotsAttempt > worlds` and open `empty.wbt`.
3. Press Ctrl+4 if "No Rendering" appears, then start the simulation with the play button.

## Results

### Annimation:
https://github.com/shreyas-chigurupati07/Transmission/assets/84034817/45e68c87-19eb-4a3e-9884-5871504256c4

### Plot:
[RRT Figure-1.pdf](https://github.com/shreyas-chigurupati07/Transmission/files/14547110/RRT.Figure-1.pdf)

## Dependencies

- WeBots R2023a
- Python
- Matplotlib


## References

- WeBots Documentation: https://cyberbotics.com/doc/guide/index
- Steven M. LaValle. Planning Algorithms. Cambridge University Press, May 2006.
