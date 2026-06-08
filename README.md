# FRC-2025-GPT (SIM-2025)

FRC robot code for Iron Panthers (Team 5026), with an embedded MCP server that lets an AI agent observe and control the robot in real time.

## What it does

This is the software that runs on the Iron Panthers competition robot for the 2025 FRC REEFSCAPE game. It controls the drivetrain, superstructure (elevator and arm), intake, claw, climb mechanism, and vision systems. On top of the standard robot code, it includes a custom HTTP server that speaks the Model Context Protocol (MCP). This lets an external AI agent (like a GPT-based system) connect over the network, read live robot state, and issue commands — drive to a position, change the superstructure state, or stop the robot entirely. The goal is to close the loop between AI reasoning and real robot actuation.

## Features

- Full swerve drive with field-relative teleop and autonomous path following (PathPlanner)
- Superstructure state machine coordinating elevator and arm with safety interlocks
- Vision-based pose estimation fusing AprilTags (PhotonLib) and odometry
- Object detection via Limelight for coral/algae game pieces
- Embedded MCP server (port 3000) for AI agent integration
- Dynamic tool registration: any subsystem can register MCP tools at runtime
- Python client (`mcp_client.py`) for connecting, listing tools, and calling them interactively
- AdvantageKit logging for full log replay and simulation
- WPILib simulation support (`./gradlew simulateJava`)
- Auto-formatter via Spotless (Google Java Format)

## How it works

```
AI Agent (Python / GPT)
      |
      | HTTP + JSON  (port 3000)
      v
  MCPServer.java  (embedded in robot process)
      |
      | calls
      v
  MCPTools registry  (dynamic, registered at startup)
      |
      +---> Drive.setTargetPosition()    -> swerve auto-align
      +---> SuperstructureController     -> elevator + arm state
      +---> RobotState.getEstimatedPose()-> current pose readout
      +---> CommandScheduler.schedule()  -> stop / other commands
```

The robot runs the standard WPILib 20 ms command loop. The MCP server runs on a background thread and enqueues actions into that loop. Tools registered at startup capture references to live subsystem objects, so tool calls take effect in the next robot cycle.

**Built-in MCP tools:**

| Tool | What it does |
|---|---|
| `GetRobotStatus` | Returns current pose, superstructure state, elevator height, arm angle, reef distances |
| `DriveToPoint` | Drives robot to (x, y, theta) on the field using PID auto-align |
| `SetSuperstructureState` | Sets elevator/arm target: STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB |
| `StopRobot` | Immediately stops all movement and clears active trajectories |

## Tech stack

| Component | Technology |
|---|---|
| Language | Java 17 |
| Robot framework | WPILib 2025 (command-based) |
| Build system | Gradle + GradleRIO |
| Drivetrain | Swerve (CTRE TalonFX, Pigeon 2 gyro) |
| Path planning | PathPlanner |
| Vision | PhotonLib (AprilTags) + Limelight (object detection) |
| Logging / replay | AdvantageKit |
| MCP server transport | HTTP/JSON (json-simple 1.1.1) |
| Python client | Python 3, requests |
| Formatter | Spotless (Google Java Format) |

## Getting started

### Prerequisites

- JDK 17
  - Windows: [oracle.com/java/technologies/downloads](https://www.oracle.com/java/technologies/downloads/)
  - Linux: `sudo apt install openjdk-17-jdk` or equivalent
- Python 3 and `pip install requests` (for the MCP client only)

### Clone

```bash
git clone git@github.com:bruce-peters/FRC-2025-GPT.git
cd FRC-2025-GPT
```

### Build

```bash
./gradlew build
```

### Run simulation

```bash
./gradlew simulateJava
```

This opens the WPILib simulation GUI and starts the robot process locally, including the MCP server on port 3000.

### Deploy to robot

```bash
./gradlew deploy
```

### Format code

```bash
./gradlew spotlessApply
```

## Usage

Once the robot (or simulation) is running, connect the Python client:

```bash
python mcp_client.py
```

Select interactive mode (option 3), then:

```
> tools          # list all registered MCP tools with schemas
> call           # prompt for tool name and parameters
  Tool name: DriveToPoint
  x (required, number): 4.5
  y (required, number): 4.0
  theta (required, number): 1.57
```

Or call tools directly from Python:

```python
from mcp_client import RobotMCPClient

client = RobotMCPClient(robot_ip="localhost", port=3000)
client.check_health()
client.initialize()
client.call_any_tool("GetRobotStatus")
client.call_any_tool("DriveToPoint", x=4.5, y=4.0, theta=1.57)
```

## Project structure

```
src/main/java/frc/robot/
  Main.java                    entry point
  Robot.java                   main robot class (LoggedRobot)
  RobotContainer.java          subsystem wiring, input bindings, MCP setup
  RobotState.java              pose estimation (EKF, vision fusion)
  Constants.java               global config
  mcp/
    MCPServer.java             embedded HTTP server (port 3000)
    MCPTools.java              dynamic tool registry
    MCPTool.java               tool interface
    MCPToolSchemas.java        JSON schema helpers
  subsystems/
    swerve/                    swerve drive (Drive.java, Module.java, gyro IOs)
    superstructure/            elevator + arm state machine
    intake/                    intake pivot + rollers + sensors
    claw/                      claw rollers
    climb/                     climb mechanism
    objectDetection/           Limelight coral/algae detection
    vision/                    AprilTag cameras
    rgb/                       LED control
  commands/                   named commands (ApproachReef, IntakeObject, ScoreL1, ...)
mcp_client.py                  Python client for AI agent integration
MCP_DYNAMIC_TOOLS.md           guide to registering new MCP tools
build.gradle                   dependencies, deploy config, Spotless
```

## Extra tools

- [FRC Driver Station (NI)](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#553883)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases) — log visualization
- [Elastic Dashboard (FRC-353)](https://github.com/Gold872/elastic-dashboard/releases)
- [Phoenix Tuner X (CTRE)](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html)

## Status

Active development. The core robot code was used in competition during the 2025 FRC REEFSCAPE season (Iron Panthers won the World Championship). The MCP/AI integration layer is an ongoing offseason addition, currently functional for simulation and local testing. The dynamic tool system is complete; expanding the tool set and wiring in an LLM-based agent loop is in progress.

---

*Licenses: WPILib (BSD-3-Clause, see WPILib-License.md), AdvantageKit (see AdvantageKit-License.md)*
