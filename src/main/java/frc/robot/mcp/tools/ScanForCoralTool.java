package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.swerve.Drive;
import org.json.simple.JSONObject;

public class ScanForCoralTool implements MCPTool {
  private final RobotContainer container;
  private final Drive swerve;
  private final ObjectDetection objectDetection;

  public ScanForCoralTool(RobotContainer container, Drive swerve, ObjectDetection objectDetection) {
    this.container = container;
    this.swerve = swerve;
    this.objectDetection = objectDetection;
  }

  @Override
  public String getName() {
    return "ScanForCoral";
  }

  @Override
  public String getDescription() {
    return "Rotates the robot up to 360 degrees while scanning for coral. Stops early if coral is detected. Returns whether coral was found during the scan.";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPTools.createInputSchema(new JSONObject());
  }

  @Override
  public JSONObject execute(JSONObject args) {
    final boolean[] coralDetected = new boolean[] {false};
    final Rotation2d[] startRotation = new Rotation2d[] {null};
    final Rotation2d[] lastRotation = new Rotation2d[] {null};
    final double[] cumulativeRotation = new double[] {0.0};

    Command scanCommand =
        new FunctionalCommand(
            // Initialize - record starting rotation
            () -> {
              startRotation[0] = RobotState.getInstance().getEstimatedPose().getRotation();
              lastRotation[0] = startRotation[0];
              coralDetected[0] = false;
              cumulativeRotation[0] = 0.0;
            },
            // Execute - continuously set target 30 degrees ahead and check for coral
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              Rotation2d currentRotation = currentPose.getRotation();

              // Calculate rotation since last execute cycle
              double deltaDegrees = currentRotation.minus(lastRotation[0]).getDegrees();
              cumulativeRotation[0] += Math.abs(deltaDegrees);
              lastRotation[0] = currentRotation;

              // Set target to 30 degrees ahead using RotateRobot formula: -degrees + 180
              Rotation2d targetRotation = currentRotation.plus(Rotation2d.fromDegrees(-30 + 180));
              Pose2d targetPose = new Pose2d(currentPose.getTranslation(), targetRotation);
              swerve.setTargetPosition(targetPose);

              // Check for coral
              if (objectDetection.coralInVision()) {
                coralDetected[0] = true;
              }
            },
            // End - clear controller
            (interrupted) -> {
              swerve.clearTargetPositionController();
            },
            // isFinished - stop when coral detected OR rotated 360 degrees
            () -> {
              if (coralDetected[0]) {
                return true;
              }

              return cumulativeRotation[0] >= 355; // 355 to account for small errors
            },
            swerve);

    container.scheduleAndWait(scanCommand);

    JSONObject result = MCPTools.successResponse();
    result.put("coral_detected", coralDetected[0]);
    result.put("total_rotation_degrees", cumulativeRotation[0]);

    if (coralDetected[0]) {
      result.put("message", "Coral detected during scan!");
      Pose2d targetCoralPose = objectDetection.getTargetPosition();
      result.put("coral_x", targetCoralPose.getX());
      result.put("coral_y", targetCoralPose.getY());
    } else {
      result.put("message", "Completed 360° scan - no coral detected");
    }

    return result;
  }
}
