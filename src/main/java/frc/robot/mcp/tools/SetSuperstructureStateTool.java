package frc.robot.mcp.tools;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPToolSchemas;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.superstructure.SuperstructureController;
import org.json.simple.JSONObject;

public class SetSuperstructureStateTool implements MCPTool {
  private final RobotContainer container;
  private final SuperstructureController superstructureController;

  public SetSuperstructureStateTool(
      RobotContainer container, SuperstructureController superstructureController) {
    this.container = container;
    this.superstructureController = superstructureController;
  }

  @Override
  public String getName() {
    return "SetSuperstructureState";
  }

  @Override
  public String getDescription() {
    return "Sets the target state of the superstructure (elevator and arm)";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPToolSchemas.getSetSuperstructureStateSchema();
  }

  @Override
  public JSONObject execute(JSONObject args) {
    String stateString = (String) args.get("state");

    // Check if still zeroing
    if (superstructureController.getSuperstructureState()
        == SuperstructureController.SuperstructureState.ZEROING) {
      JSONObject error = new JSONObject();
      error.put("success", false);
      error.put(
          "error",
          "Superstructure is still zeroing. Please wait for zeroing to complete before setting state.");
      error.put("current_state", "ZEROING");
      return error;
    }

    // Try to parse the state
    SuperstructureController.SuperstructureState targetState;
    try {
      targetState = SuperstructureController.SuperstructureState.valueOf(stateString.toUpperCase());
    } catch (IllegalArgumentException e) {
      JSONObject error = new JSONObject();
      error.put("success", false);
      error.put("error", "Invalid state: " + stateString);
      error.put("valid_states", "STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB");
      return error;
    }

    // Prevent setting to ZEROING state
    if (targetState == SuperstructureController.SuperstructureState.ZEROING) {
      JSONObject error = new JSONObject();
      error.put("success", false);
      error.put("error", "Cannot manually set ZEROING state");
      return error;
    }

    // Schedule the command to set the state and wait for completion
    Command setSuperstructureCommand =
        superstructureController.setSuperstructureStateCommand(targetState);
    container.scheduleAndWait(setSuperstructureCommand);

    JSONObject result = MCPTools.successResponse();
    result.put("message", "Setting superstructure state to: " + targetState.toString());
    result.put("state", targetState.toString());

    return result;
  }
}
