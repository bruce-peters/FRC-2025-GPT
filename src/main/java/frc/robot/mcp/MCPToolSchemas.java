package frc.robot.mcp;

import org.json.simple.JSONObject;

public class MCPToolSchemas {

  public static JSONObject getDriveToPointScheama() {
    JSONObject properties = new JSONObject();
    properties.put("x", MCPTools.createNumberParam("X coordinate in meters"));
    properties.put("y", MCPTools.createNumberParam("Y coordinate in meters"));
    properties.put("theta", MCPTools.createNumberParam("Orientation in degrees"));

    return MCPTools.createInputSchema(properties, "x", "y", "theta");
  }

  public static JSONObject getRotateRobotSchema() {
    JSONObject properties = new JSONObject();
    properties.put(
        "degrees",
        MCPTools.createNumberParam(
            "Degrees to rotate. Positive = counterclockwise, negative = clockwise"));

    return MCPTools.createInputSchema(properties, "degrees");
  }

  public static JSONObject getSetSuperstructureStateSchema() {
    JSONObject properties = new JSONObject();
    properties.put(
        "state",
        MCPTools.createStringParam(
            "Target superstructure state. Valid values: STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB"));

    return MCPTools.createInputSchema(properties, "state");
  }

  public static JSONObject getMoveRelativeSchema() {
    JSONObject properties = new JSONObject();
    properties.put(
        "forward",
        MCPTools.createNumberParam("Distance to move forward in meters (negative = backward)"));
    properties.put(
        "left", MCPTools.createNumberParam("Distance to move left in meters (negative = right)"));

    return MCPTools.createInputSchema(properties, "forward", "left");
  }
}
