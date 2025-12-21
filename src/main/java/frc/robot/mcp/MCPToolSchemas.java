package frc.robot.mcp;

import org.json.simple.JSONObject;

public class MCPToolSchemas {

  public static JSONObject getDriveToPointScheama() {
    JSONObject properties = new JSONObject();
    properties.put("x", MCPTools.createNumberParam("X coordinate in meters"));
    properties.put("y", MCPTools.createNumberParam("Y coordinate in meters"));
    properties.put("theta", MCPTools.createNumberParam("Orientation in radians"));

    return MCPTools.createInputSchema(properties, "x", "y", "theta");
  }

  public static JSONObject getSetSuperstructureStateSchema() {
    JSONObject properties = new JSONObject();
    properties.put(
        "state",
        MCPTools.createStringParam(
            "Target superstructure state. Valid values: STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB"));

    return MCPTools.createInputSchema(properties, "state");
  }
}
