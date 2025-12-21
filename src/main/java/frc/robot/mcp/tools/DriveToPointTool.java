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

    final Pose2d[] lastPose = new Pose2d[] {null};
    final int[] stuckCounter = new int[] {0};
    final boolean[] reachedTarget = new boolean[] {false};
    final int STUCK_THRESHOLD = 50; // 50 cycles (~1 second at 50Hz) without progress
    final double STUCK_DISTANCE_THRESHOLD = 0.02; // 2cm movement threshold

    Command driveCommand =
        new FunctionalCommand(
            () -> {
              swerve.setTargetPosition(targetPose);
              lastPose[0] = RobotState.getInstance().getEstimatedPose();
              stuckCounter[0] = 0;
              reachedTarget[0] = false;
            },
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

              // Check if robot is making progress
              if (lastPose[0] != null) {
                double distanceMoved =
                    currentPose.getTranslation().getDistance(lastPose[0].getTranslation());
                if (distanceMoved < STUCK_DISTANCE_THRESHOLD) {
                  stuckCounter[0]++;
                } else {
                  stuckCounter[0] = 0; // Reset if robot is moving
                }
              }

              lastPose[0] = currentPose;
            },
            (interrupted) -> swerve.clearTargetPositionController(),
            () -> {
              // Check if we're close enough to the target
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              double distanceError =
                  currentPose.getTranslation().getDistance(targetPose.getTranslation());
              double angleError =
                  Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

              if (distanceError < 0.05 && angleError < Math.toRadians(5)) {
                reachedTarget[0] = true;
                return true; // 5cm and 5 degrees tolerance
              }

              // Check if robot is stuck
              return stuckCounter[0] >= STUCK_THRESHOLD;
            },
            swerve);

    container.scheduleAndWait(driveCommand);

    Pose2d finalPose = RobotState.getInstance().getEstimatedPose();
    JSONObject result = MCPTools.successResponse();

    if (reachedTarget[0]) {
      result.put("message", "Successfully reached target position");
      result.put("reached_target", true);
    } else if (stuckCounter[0] >= STUCK_THRESHOLD) {
      result.put("success", false);
      result.put("message", "Robot appears to be stuck and unable to reach target position");
      result.put("reached_target", false);

      double distanceError = finalPose.getTranslation().getDistance(targetPose.getTranslation());
      double angleError =
          Math.abs(finalPose.getRotation().minus(targetPose.getRotation()).getDegrees());
      result.put("distance_error_meters", distanceError);
      result.put("angle_error_degrees", angleError);
    }

    // Always include current position in response
    result.put("current_x", finalPose.getX());
    result.put("current_y", finalPose.getY());
    result.put("current_rotation_degrees", finalPose.getRotation().getDegrees());
    result.put("target_x", x);
    result.put("target_y", y);
    result.put("target_rotation_degrees", thetaDegrees);

    return result;
  }
}
