package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPToolSchemas;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.swerve.Drive;
import org.json.simple.JSONObject;

public class MoveRelativeTool implements MCPTool {
  private final RobotContainer container;
  private final Drive swerve;

  public MoveRelativeTool(RobotContainer container, Drive swerve) {
    this.container = container;
    this.swerve = swerve;
  }

  @Override
  public String getName() {
    return "MoveRelative";
  }

  @Override
  public String getDescription() {
    return "Moves the robot relative to its current position and orientation. Forward is in the direction the robot is facing.";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPToolSchemas.getMoveRelativeSchema();
  }

  @Override
  public JSONObject execute(JSONObject args) {
    // Validate parameters
    if (args.get("forward") == null || args.get("left") == null) {
      JSONObject error = new JSONObject();
      error.put("success", false);
      error.put("error", "Missing required parameters: forward and left are required");
      error.put("received_args", args.toString());
      return error;
    }

    double forward = ((Number) args.get("forward")).doubleValue();
    double left = ((Number) args.get("left")).doubleValue();

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    // Calculate target position in robot-relative coordinates
    // Forward = along robot's current heading, Left = perpendicular to heading
    Translation2d relativeTranslation = new Translation2d(forward, left);
    Translation2d fieldRelativeTranslation =
        relativeTranslation.rotateBy(currentPose.getRotation());
    Translation2d targetTranslation = currentPose.getTranslation().plus(fieldRelativeTranslation);

    // Keep the same rotation
    Pose2d targetPose =
        new Pose2d(targetTranslation, currentPose.getRotation().plus(Rotation2d.fromDegrees(180)));

    Command moveCommand =
        new FunctionalCommand(
            () -> swerve.setTargetPosition(targetPose),
            () -> {},
            (interrupted) -> swerve.clearTargetPositionController(),
            () -> {
              // Check if we're close enough to the target
              Pose2d currentPosition = RobotState.getInstance().getEstimatedPose();
              double distanceError =
                  currentPosition.getTranslation().getDistance(targetPose.getTranslation());
              return distanceError < 0.05; // 5cm tolerance
            },
            swerve);

    container.scheduleAndWait(moveCommand);

    JSONObject result = MCPTools.successResponse();
    result.put("message", "Moved forward: " + forward + "m, left: " + left + "m");
    result.put("target_x", targetPose.getX());
    result.put("target_y", targetPose.getY());
    return result;
  }
}
