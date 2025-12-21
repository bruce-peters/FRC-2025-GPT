package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPToolSchemas;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.swerve.Drive;
import org.json.simple.JSONObject;

public class RotateRobotTool implements MCPTool {
  private final RobotContainer container;
  private final Drive swerve;

  public RotateRobotTool(RobotContainer container, Drive swerve) {
    this.container = container;
    this.swerve = swerve;
  }

  @Override
  public String getName() {
    return "RotateRobot";
  }

  @Override
  public String getDescription() {
    return "Rotates the robot by a specified number of degrees relative to its current orientation";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPToolSchemas.getRotateRobotSchema();
  }

  @Override
  public JSONObject execute(JSONObject args) {
    double degrees = ((Number) args.get("degrees")).doubleValue();

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Rotation2d newRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(-degrees + 180));
    Pose2d targetPose = new Pose2d(currentPose.getTranslation(), newRotation);

    Command rotateCommand =
        new FunctionalCommand(
            () -> swerve.setTargetPosition(targetPose),
            () -> {},
            (interrupted) -> swerve.clearTargetPositionController(),
            () -> {
              // Check if we're close enough to the target rotation
              Pose2d currentPosition = RobotState.getInstance().getEstimatedPose();
              double angleError =
                  Math.abs(
                      currentPosition.getRotation().minus(targetPose.getRotation()).getDegrees()
                          - 180);
              return angleError < 5; // 5 degrees tolerance
            },
            swerve);

    container.scheduleAndWait(rotateCommand);

    JSONObject result = MCPTools.successResponse();
    result.put(
        "message",
        "Rotating robot by "
            + degrees
            + "° to new heading: "
            + targetPose.getRotation().getDegrees()
            + "°");
    return result;
  }
}
