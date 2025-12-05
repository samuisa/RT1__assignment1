# Reactive Turtlesim Project - Collision Avoidance

This project implements a reactive control system in ROS 2 (Python and C++) for two turtles in the `turtlesim` simulator. The main goal is to prevent collisions between the turtles and with the tank walls, acting as a safety filter over the raw motion commands provided by the user.

---

## System Architecture

The system consists of three ROS 2 nodes, each with a specific role:

| Node | Language | Role | Topic/Service |
| :--- | :--- | :--- | :--- |
| **`spawn.py`** | Python | Initialization of the second turtle. | Client `/spawn` |
| **`ui_node.cpp`** | C++ | Interface for desired motion commands. | Publisher to `/turtleX/raw_cmd_vel` |
| **`distance_node.cpp`** | C++ | Safety filter and collision prevention logic. | Subscriber/Publisher to command and pose topics. |


```bash
.
└── assignment1_rt
    ├── CMakeLists.txt
    ├── package.xml
    ├── scripts
    │   └── spawn.py
    └── src
        ├── distance_node.cpp
        └── ui_node.cpp
```

---

## 1. `spawn.py` (Turtle Spawner)

This Python client node is responsible for adding a second turtle to the simulator.

* **Action:** Calls the `/spawn` service of `turtlesim`.
* **Name:** Assigns the name **`turtle2`**.
* **Position:** Places the turtle at coordinates **(x=7.0, y=7.0)**.
* **Mechanism:** Uses an asynchronous call (`call_async`) to avoid blocking the node while waiting for the response.

---

## 2. `ui_node.cpp` (User Interface)

This C++ node provides a command-line interface for sending the **desired** (raw) movement commands to the turtles.

* **Input:** The user selects the turtle (`turtle1` or `turtle2`) and enters the desired linear velocity (`linear.x`) and angular velocity (`angular.z`).
* **Output Topic:** Publishes the command to the **`/turtleX/raw_cmd_vel`** topic.
* **Duration:** The command is maintained for **1 second**, after which the node automatically sends a zero-velocity command to stop the robot, prompting the user for a new input.

---

## 3. `distance_node.cpp` (Reactive Safety Control)

This C++ node is the brain of the safety system. It intercepts the raw commands and filters them before they reach the turtle's motors, ensuring safety.

### Control Flow

1.  **Subscriptions:**
    * Receives the current pose from `/turtleX/pose`.
    * Receives the desired command from `/turtleX/raw_cmd_vel`.
2.  **Publisher:**
    * Publishes the filtered command to the `/turtleX/cmd_vel` topic.
3.  **Frequency:** The control loop runs at **20 Hz** (every $50 \text{ ms}$).

### Safety Logic

The node implements the `handle_turtle` function for each robot, applying the following rules:

1.  **Command Limiting:**
    * Limits linear velocity to $\pm 5.0$.
    * Limits angular velocity to $\pm 5.0$.
2.  **Critical Zone Definition:**
    * **Collision:** The distance between the two turtles is $\leq 1.0$.
    * **Wall:** A turtle's position is $\leq 1.0$ or $\geq 10.0$ (tank boundary).
3.  **Collision Prevention (Safety Stop):**

    When a turtle is in a Critical Zone (`Collision` or `Wall`), its **linear velocity** command is **immediately blocked** (`cmd.linear.x = 0.0`), allowing only angular movement.

    To be able to receive a **linear velocity** command again (to exit the blocked state), the **Safety Prediction** condition must be met:
    
    * **Prediction:** If the turtle is safe OR if it's in a critical zone but is about to receive a movement command, the node calculates its future position after $50 \text{ ms}$ (`dt_`). The linear velocity command is **preventively blocked** ONLY if the calculated future position falls into a Critical Zone.

    In summary, linear velocity is only permitted if the turtle about to move does not displace to a position where:
    * The distance from the other turtle is $\leq 1.0$.
    * The distance from the wall is $\leq 1.0$.

#### How to run the simulation

1.  Open 4 different shell terminals.
2.  Initialize the ROS 2 environment (adjust the ROS distribution name if necessary):
    ```bash
    source /opt/ros/jazzy/setup.bash
    ```
3.  Build the workspace:
    ```bash
    colcon build
    ```
    ```bash
    source install/local_setup.sh
    ```
4. Run the nodes, the spawner, and the turtlesim simulator:
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    ```bash
    ros2 run assignment1_rt spawn.py
    ```
    ```bash
    ros2 run assignment1_rt distance
    ```
    ```bash
    ros2 run assignment1_rt ui
    ```
