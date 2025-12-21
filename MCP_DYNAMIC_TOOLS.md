# MCP Dynamic Tools System

## Overview
The MCP server now uses a fully dynamic tool registration system. Tools can be registered at runtime from anywhere in your robot code, and the Python client automatically adapts to any registered tools.

## Server Changes

### MCPServer.java
- **Removed** resources endpoint (not needed yet)
- **Removed** `handleResourcesList()` method
- **Updated** capabilities to not advertise resources

### MCPTools.java
- **Removed** `getResourcesList()` method
- Dynamic tool registration via `HashMap<String, MCPTool>`
- Helper methods for creating schemas:
  - `createNumberParam(description)`
  - `createStringParam(description)`
  - `createInputSchema(properties, required...)`

### MCPTool Interface
```java
public interface MCPTool {
    String getName();
    String getDescription();
    JSONObject getInputSchema();
    JSONObject execute(JSONObject arguments) throws Exception;
}
```

## Registering Tools

### Simple Example
```java
MCPTools.registerTool(
    "get_robot_pose",
    "Gets the current robot pose",
    MCPTools.createInputSchema(new JSONObject()),
    (args) -> {
        Pose2d pose = RobotContainer.swerve.getPose();
        JSONObject result = new JSONObject();
        result.put("x", pose.getX());
        result.put("y", pose.getY());
        result.put("rotation", pose.getRotation().getDegrees());
        return result;
    }
);
```

### With Parameters
```java
JSONObject properties = new JSONObject();
properties.put("xSpeed", MCPTools.createNumberParam("Forward speed in m/s"));
properties.put("ySpeed", MCPTools.createNumberParam("Strafe speed in m/s"));
properties.put("rotSpeed", MCPTools.createNumberParam("Rotation speed in rad/s"));
properties.put("duration", MCPTools.createNumberParam("Duration in seconds"));

MCPTools.registerTool(
    "drive_robot",
    "Drives the robot with specified velocities",
    MCPTools.createInputSchema(properties, "xSpeed", "ySpeed", "rotSpeed", "duration"),
    (args) -> {
        double xSpeed = ((Number) args.get("xSpeed")).doubleValue();
        double ySpeed = ((Number) args.get("ySpeed")).doubleValue();
        double rotSpeed = ((Number) args.get("rotSpeed")).doubleValue();
        double duration = ((Number) args.get("duration")).doubleValue();

        // Drive robot...

        JSONObject result = new JSONObject();
        result.put("success", true);
        return result;
    }
);
```

## Python Client Changes

### New Features
- **`call_any_tool(tool_name, **kwargs)`**: Generic method to call any tool
- **Enhanced `list_tools()`**: Shows parameter schemas with types and descriptions
- **Dynamic interactive mode**: Prompts for parameters based on tool schema

### Usage Examples

```python
# Create client and connect
client = RobotMCPClient()
client.check_health()
client.initialize()

# List all available tools (shows schemas)
client.list_tools()

# Call any tool dynamically
client.call_any_tool("get_robot_pose")
client.call_any_tool("drive_robot", xSpeed=0.5, ySpeed=0.0, rotSpeed=0.0, duration=1.0)
```

### Interactive Mode
```
> tools           # Lists all tools with schemas
> call            # Prompts for tool name and parameters
  Tool name: drive_robot
  xSpeed (required, number): 0.5
  ySpeed (required, number): 0.0
  rotSpeed (required, number): 0.0
  duration (required, number): 1.0
```

## Benefits
- ✅ No hardcoded tool lists
- ✅ Register tools from RobotContainer with subsystem access
- ✅ Client automatically adapts to available tools
- ✅ AI agents can discover and use tools dynamically
- ✅ Easy to add new tools without modifying server endpoints
