# FRC-2025-GPT (SIM-2025)

FRC robot code for Iron Panthers (Team 5026), with an embedded MCP-style server that lets an AI agent observe and control the robot in real time.

> Fork of [Iron-Panthers/FRC-2025-Sprint](https://github.com/Iron-Panthers/FRC-2025-Sprint), the Iron Panthers' 2025 robot/simulation codebase. The robot code (drive, superstructure, intake, vision, object detection) is the team's work; the MCP server, its tools, and the Python client (`src/main/java/frc/robot/mcp/`, `configureMCP()` in `RobotContainer.java`, `mcp_client.py`) were added in this fork.

## What it does

This is the software that runs on the Iron Panthers competition robot for the 2025 FRC REEFSCAPE game. It controls the drivetrain, superstructure (elevator and arm), intake, claw, climb mechanism, and vision systems. On top of the standard robot code, it includes a custom HTTP server modeled on the Model Context Protocol (MCP): `initialize`, `tools/list` and `tools/call` endpoints that speak plain JSON. This lets an external AI agent (like a GPT-based system) connect over the network, read live robot state, and issue commands — drive to a position, change the superstructure state, or stop the robot entirely. The goal is to close the loop between AI reasoning and real robot actuation.

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
  LLM agent (GPT)                     mcp_client.py              curl / any HTTP client
  openai_robot_control.py             (interactive CLI,
  [feat/mcp-robot branch]              Python API)
          |                                 |                            |
          +---------------------------------+----------------------------+
                                            |
                              HTTP + JSON, port 3000
                                            |
  +-----------------------------------------v------------------------------------------+
  |  Robot process  (real roboRIO, or desktop via ./gradlew simulateJava)              |
  |                                                                                    |
  |  MCPServer.java  --->  MCPTools registry  --->  DriveToPoint     -> Drive (swerve) |
  |  (HttpServer,          (registered in           GetRobotStatus   -> RobotState,    |
  |   4-thread pool)        RobotContainer           SetSuperstructureState            |
  |                         .configureMCP())        StopRobot        -> Drive          |
  |                                                                                    |
  |  WPILib command loop (20 ms)  <---  commands scheduled by the tools                |
  |  AdvantageKit Logger  --->  NT4Publisher (NetworkTables, port 5810)                |
  +------------------------------------------------------------------------------------+
                                            |
                                  NetworkTables 4
                                            v
                                     AdvantageScope
                              (live field / mechanism view)
```

- The MCP server is a `com.sun.net.httpserver.HttpServer` started from `RobotContainer.configureMCP()`. Each tool is a lambda that captures live subsystem objects; tools that move the robot build a WPILib `Command` and `schedule()` it, so the motion runs in the next 20 ms robot cycles.
- NetworkTables is used only for telemetry (AdvantageKit → AdvantageScope). The AI path is plain HTTP.
- The transport is JSON over plain HTTP paths, not the JSON-RPC transport used by stock MCP clients (Claude Desktop, etc.), so those clients cannot connect directly; use `mcp_client.py`, the agent script on `feat/mcp-robot`, or `curl`.

**Built-in MCP tools (on `dev`):**

| Tool | What it does |
|---|---|
| `GetRobotStatus` | Returns current pose, superstructure state, elevator height, arm angle, reef distances |
| `DriveToPoint` | Drives robot to (x, y, theta) on the field using PID auto-align |
| `SetSuperstructureState` | Sets elevator/arm target: STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB |
| `StopRobot` | Immediately stops all movement and clears active trajectories |

**HTTP endpoints (`MCPServer.java`):**

| Method | Path | Body / result |
|---|---|---|
| GET | `/health` | `{"status":"healthy","timestamp":...}` |
| POST | `/mcp/v1/initialize` | protocol version, server info, capabilities |
| GET | `/mcp/v1/tools/list` | `{"tools":[{name, description, inputSchema}, ...]}` |
| POST | `/mcp/v1/tools/call` | body `{"name": "<tool>", "arguments": {...}}` → `{"content": {...}}` |
| GET | `/mcp/v1/prompts/list` | `{"prompts":[]}` |

### LLM agent (on the `feat/mcp-robot` branch)

The GPT-4o-mini agent loop, `openai_robot_control.py`, and the larger tool set it uses (`ScanForCoral`, `IntakeClosestObject`, `MoveRelative`, `RotateRobot`, plus the four above, as classes in `src/main/java/frc/robot/mcp/tools/`) live on the [`feat/mcp-robot`](https://github.com/bruce-peters/FRC-2025-GPT/tree/feat/mcp-robot) branch and are not merged into `dev` yet. That script converts the robot's tool list into OpenAI function definitions, injects `GetRobotStatus` output before every model turn, and executes the tool calls the model makes. It reads `OPENAI_API_KEY` (and optionally `ROBOT_URL`, default `http://localhost:3000`) from a `.env` file (copy `.env.example` to `.env`; `.env` is git-ignored) and needs `pip install openai python-dotenv requests`. Calls to the OpenAI API are billed to that key.

## Run it

Tested on Linux with OpenJDK 17.0.20 and Python 3.13. Commands are for macOS/Linux; on Windows use `gradlew.bat` and `.venv\Scripts\python`.

### Prerequisites

- **JDK 17** (required by WPILib 2025 / GradleRIO 2025.2.1). The [WPILib 2025 installer](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) bundles one; otherwise:
  - Windows: [oracle.com/java/technologies/downloads](https://www.oracle.com/java/technologies/downloads/)
  - Linux: `sudo apt install openjdk-17-jdk` or equivalent
  - Make sure `JAVA_HOME` points at it (`java -version` should print 17).
- Internet access on the first build: Gradle downloads WPILib and the vendor libraries in `vendordeps/` (about 1 GB, cached in `~/.gradle` afterwards).
- **Python 3** for the MCP client.

### 1. Clone and build

```bash
git clone https://github.com/bruce-peters/FRC-2025-GPT.git
cd FRC-2025-GPT
./gradlew build
```

The build runs `spotlessApply` first, so it may reformat Java/JSON/Markdown files you have edited.

### 2. Start the robot simulation

```bash
./gradlew simulateJava
```

This starts the robot program on your computer with the WPILib Simulation GUI. Startup is done when the console prints:

```
MCP Server started. Listening for AI agent requests...
MCP: Registered tool 'DriveToPoint'
...
********** Robot program startup complete **********
```

The robot starts **disabled**. In the Simulation GUI's *Robot State* panel, click **Teleoperated** (or Autonomous) to enable it. While disabled, tools still answer but nothing moves: `DriveToPoint` is accepted without motion, and the superstructure stays in `ZEROING` (the elevator zeroes by driving down until it sees a current spike), so `SetSuperstructureState` returns an error until the robot is enabled.

### 3. Talk to the robot

In a second terminal, check the server:

```bash
curl http://localhost:3000/health
curl http://localhost:3000/mcp/v1/tools/list
curl -X POST http://localhost:3000/mcp/v1/tools/call \
  -H 'Content-Type: application/json' \
  -d '{"name": "GetRobotStatus", "arguments": {}}'
```

Then the Python client:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python mcp_client.py
```

It asks for a mode: `1` connects and lists tools, `2` reads the pose and (after a y/n prompt) drives 0.5 m with `DriveToPoint` then calls `StopRobot`, `3` is interactive:

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
client.call_any_tool("StopRobot")
```

The client connects to `localhost:3000`. For a real robot, pass the roboRIO address instead, e.g. `RobotMCPClient(robot_ip="10.50.26.2")`.

### 4. Watch it in AdvantageScope (optional)

In simulation, AdvantageKit publishes all logged data over NetworkTables 4 (`NT4Publisher`). Open [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases) and use *File → Connect to Simulator* to watch the robot pose and other logged outputs (e.g. PathPlanner targets) live while the AI drives. The `advantage_scope_files/Robot_SprintBot/` folder is a custom 3D robot model; copy it into AdvantageScope's assets folder (*Help → Show Assets Folder*, see [custom assets](https://docs.advantagescope.org/more-features/custom-assets)) to use it in the 3D Field view. *(AdvantageScope steps not tested in this environment.)*

**Log replay:** in `SIM` mode the code writes no `.wpilog` file; logs come from the real robot (`WPILOGWriter` to the USB stick). To replay one with AdvantageKit, set `Constants.ROBOT_TYPE = RobotType.COMP` and run `./gradlew simulateJava`; `Robot.java` then switches to `REPLAY` mode, finds the log with `LogFileUtil.findReplayLog()`, and writes a `_sim` copy next to it. See the [AdvantageKit replay docs](https://docs.advantagekit.org/). *(Not tested here.)*

### Deploy to a robot

`Constants.ROBOT_TYPE` is `RobotType.SIM` on this branch, which selects simulated hardware even on a roboRIO. Set it to `RobotType.COMP` first, then:

```bash
./gradlew deploy
```

The team number (5026) comes from `.wpilib/wpilib_preferences.json`. *(Needs roboRIO hardware; not tested here.)*

### Format code

```bash
./gradlew spotlessApply
```

## Project structure

```
src/main/java/frc/robot/
  Main.java                    entry point
  Robot.java                   main robot class (LoggedRobot), AdvantageKit setup
  RobotContainer.java          subsystem wiring, input bindings, MCP setup (configureMCP)
  RobotState.java              pose estimation (EKF, vision fusion)
  Constants.java               global config (ROBOT_TYPE: SIM or COMP)
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
requirements.txt               Python dependencies for mcp_client.py
.env.example                   OPENAI_API_KEY placeholder for the GPT agent (feat/mcp-robot)
MCP_DYNAMIC_TOOLS.md           guide to registering new MCP tools
advantage_scope_files/         custom AdvantageScope robot model
build.gradle                   dependencies, deploy config, Spotless
```

## Notes and limitations

- Tool handlers run on the HTTP server's thread pool, not the robot's main loop, and call `Command.schedule()` from there. WPILib's `CommandScheduler` is not thread-safe, so concurrent tool calls could race with the 20 ms loop. Fine for interactive testing; for competition use, queue tool actions and drain the queue in `robotPeriodic()`.
- `DriveToPoint` drives straight with the auto-align PID controller; it does not path-plan around the reef.
- The MCP server has no authentication. Anyone on the robot network can call tools.

## Extra tools

- [FRC Driver Station (NI)](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html#553883)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases) — log visualization
- [Elastic Dashboard (FRC-353)](https://github.com/Gold872/elastic-dashboard/releases)
- [Phoenix Tuner X (CTRE)](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html)

## Status

Active development. The core robot code was used in competition during the 2025 FRC REEFSCAPE season (Iron Panthers won the World Championship). The MCP/AI integration layer is an ongoing offseason addition, currently functional for simulation and local testing. The dynamic tool system is complete. The LLM agent loop and the expanded tool set are on the `feat/mcp-robot` branch and not yet merged into `dev`.

---

*Licenses: WPILib (BSD-3-Clause, see WPILib-License.md), AdvantageKit (see AdvantageKit-License.md)*
