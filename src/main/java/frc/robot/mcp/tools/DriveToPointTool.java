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

public class DriveToPointTool implements MCPTool {
  private final RobotContainer container;
  private final Drive swerve;

  public DriveToPointTool(RobotContainer container, Drive swerve) {
    this.container = container;
    this.swerve = swerve;
  }

  @Override
  public String getName() {
    return "DriveToPoint";
  }

  @Override
  public String getDescription() {
    return "Drives the robot to a specified field point";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPToolSchemas.getDriveToPointScheama();
  }

  @Override
  public JSONObject execute(JSONObject args) {
    double x = ((Number) args.get("x")).doubleValue();
    double y = ((Number) args.get("y")).doubleValue();
    double thetaDegrees = ((Number) args.get("theta")).doubleValue();

    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegrees));

    Command driveCommand =
        new FunctionalCommand(
            () -> swerve.setTargetPosition(targetPose),
            () -> {},
            (interrupted) -> swerve.clearTargetPositionController(),
            () -> {
              // Check if we're close enough to the target
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              double distanceError =
                  currentPose.getTranslation().getDistance(targetPose.getTranslation());
              double angleError =
                  Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
              return distanceError < 0.05
                  && angleError < Math.toRadians(5); // 5cm and 5 degrees tolerance
            },
            swerve);

    container.scheduleAndWait(driveCommand);

    JSONObject result = MCPTools.successResponse();
    result.put(
        "message", "Driving to position: x=" + x + ", y=" + y + ", theta=" + thetaDegrees + "°");
    return result;
  }
}
