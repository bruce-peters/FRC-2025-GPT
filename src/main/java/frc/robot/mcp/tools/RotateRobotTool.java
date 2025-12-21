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

    final Pose2d[] lastPose = new Pose2d[] {null};
    final int[] stuckCounter = new int[] {0};
    final boolean[] reachedTarget = new boolean[] {false};
    final int STUCK_THRESHOLD = 50; // 50 cycles (~1 second at 50Hz) without progress
    final double STUCK_ROTATION_THRESHOLD = 1.0; // 1 degree rotation threshold

    Command rotateCommand =
        new FunctionalCommand(
            () -> {
              swerve.setTargetPosition(targetPose);
              lastPose[0] = RobotState.getInstance().getEstimatedPose();
              stuckCounter[0] = 0;
              reachedTarget[0] = false;
            },
            () -> {
              Pose2d current = RobotState.getInstance().getEstimatedPose();

              // Check if robot is making progress
              if (lastPose[0] != null) {
                double rotationChange =
                    Math.abs(current.getRotation().minus(lastPose[0].getRotation()).getDegrees());
                if (rotationChange < STUCK_ROTATION_THRESHOLD) {
                  stuckCounter[0]++;
                } else {
                  stuckCounter[0] = 0; // Reset if robot is rotating
                }
              }

              lastPose[0] = current;
            },
            (interrupted) -> swerve.clearTargetPositionController(),
            () -> {
              // Check if we're close enough to the target rotation
              Pose2d currentPosition = RobotState.getInstance().getEstimatedPose();
              double angleError =
                  Math.abs(
                      currentPosition.getRotation().minus(targetPose.getRotation()).getDegrees());

              // Normalize angle error to [-180, 180]
              while (angleError > 180) angleError -= 360;
              while (angleError < -180) angleError += 360;

              if (Math.abs(angleError) < 5) {
                reachedTarget[0] = true;
                return true; // 5 degrees tolerance
              }

              // Check if robot is stuck
              return stuckCounter[0] >= STUCK_THRESHOLD;
            },
            swerve);

    container.scheduleAndWait(rotateCommand);

    Pose2d finalPose = RobotState.getInstance().getEstimatedPose();
    double finalAngleError =
        Math.abs(finalPose.getRotation().minus(targetPose.getRotation()).getDegrees());

    JSONObject result = MCPTools.successResponse();

    if (reachedTarget[0]) {
      result.put("message", "Successfully rotated to target heading");
      result.put("reached_target", true);
    } else if (stuckCounter[0] >= STUCK_THRESHOLD) {
      result.put("success", false);
      result.put("message", "Robot got stuck while rotating");
      result.put("reached_target", false);
      result.put("angle_error_degrees", finalAngleError);
    }

    result.put("target_rotation_degrees", targetPose.getRotation().getDegrees());
    result.put("final_rotation_degrees", finalPose.getRotation().getDegrees());
    result.put("rotation_change_degrees", degrees);

    return result;
  }
}
