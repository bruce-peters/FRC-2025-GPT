package frc.robot.mcp.tools;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.swerve.Drive;
import org.json.simple.JSONObject;

public class StopRobotTool implements MCPTool {
  private final Drive swerve;

  public StopRobotTool(Drive swerve) {
    this.swerve = swerve;
  }

  @Override
  public String getName() {
    return "StopRobot";
  }

  @Override
  public String getDescription() {
    return "Stops all robot movement immediately. Stops both the drivebase and superstructure (elevator and arm).";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPTools.createInputSchema(new JSONObject());
  }

  @Override
  public JSONObject execute(JSONObject args) {
    // Stop the drivebase by setting speeds to zero
    swerve.setTrajectorySpeeds(new ChassisSpeeds(0, 0, 0));
    swerve.clearTargetPositionController();
    swerve.clearHeadingControl();

    JSONObject result = MCPTools.successResponse();
    result.put("message", "Robot stopped");
    return result;
  }
}
