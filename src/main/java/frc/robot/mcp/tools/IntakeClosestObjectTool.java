package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.swerve.Drive;
import org.json.simple.JSONObject;

public class IntakeClosestObjectTool implements MCPTool {
  private final RobotContainer container;
  private final Drive swerve;
  private final ObjectDetection objectDetection;
  private final IntakeController intakeController;

  public IntakeClosestObjectTool(
      RobotContainer container,
      Drive swerve,
      ObjectDetection objectDetection,
      IntakeController intakeController) {
    this.container = container;
    this.swerve = swerve;
    this.objectDetection = objectDetection;
    this.intakeController = intakeController;
  }

  @Override
  public String getName() {
    return "IntakeClosestObject";
  }

  @Override
  public String getDescription() {
    return "Drives the robot to the closest detected coral object and intakes it. Uses object detection camera to locate the coral, then approaches and picks it up automatically.";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPTools.createInputSchema(new JSONObject());
  }

  @Override
  public JSONObject execute(JSONObject args) {
    // Check if object is detected
    if (!objectDetection.coralInVision()) {
      JSONObject error = new JSONObject();
      error.put("success", false);
      error.put("error", "No coral detected in camera view");
      return error;
    }

    // Capture the INITIAL target position - we won't chase other coral
    final Pose2d initialTargetPose = objectDetection.getTargetPosition();
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    double initialDistance =
        currentPose.getTranslation().getDistance(initialTargetPose.getTranslation());

    final Pose2d[] lastPose = new Pose2d[] {null};
    final int[] stuckCounter = new int[] {0};
    final boolean[] reachedTarget = new boolean[] {false};
    final int STUCK_THRESHOLD = 50; // 50 cycles (~1 second at 50Hz) without progress
    final double STUCK_DISTANCE_THRESHOLD = 0.02; // 2cm movement threshold

    // Create a custom command that:
    // 1. Starts the intake
    // 2. Drives to the INITIAL target position (doesn't chase new coral)
    // 3. Finishes when within 30cm of the INITIAL target OR if stuck
    Command intakeCommand =
        new FunctionalCommand(
            // Initialize - start intake and set target
            () -> {
              intakeController.setTargetState(IntakeController.IntakeState.INTAKE);
              swerve.setTargetPosition(initialTargetPose);
              lastPose[0] = RobotState.getInstance().getEstimatedPose();
              stuckCounter[0] = 0;
              reachedTarget[0] = false;
            },
            // Execute - maintain target position and check for stuck
            () -> {
              Pose2d current = RobotState.getInstance().getEstimatedPose();

              // Check if robot is making progress
              if (lastPose[0] != null) {
                double distanceMoved =
                    current.getTranslation().getDistance(lastPose[0].getTranslation());
                if (distanceMoved < STUCK_DISTANCE_THRESHOLD) {
                  stuckCounter[0]++;
                } else {
                  stuckCounter[0] = 0; // Reset if robot is moving
                }
              }

              lastPose[0] = current;

              // Keep driving to the initial target, ignore any other coral
              swerve.setTargetPosition(initialTargetPose);
            },
            // End - clear drive controller
            (interrupted) -> {
              swerve.clearTargetPositionController();
            },
            // isFinished - stop when within 30cm of INITIAL target OR stuck
            () -> {
              Pose2d current = RobotState.getInstance().getEstimatedPose();
              double distanceToInitialTarget =
                  current.getTranslation().getDistance(initialTargetPose.getTranslation());

              if (distanceToInitialTarget < 0.3) {
                reachedTarget[0] = true;
                return true; // Within 30cm
              }

              // Check if robot is stuck
              return stuckCounter[0] >= STUCK_THRESHOLD;
            },
            swerve,
            intakeController);

    container.scheduleAndWait(intakeCommand);

    Pose2d finalPose = RobotState.getInstance().getEstimatedPose();
    double finalDistance =
        finalPose.getTranslation().getDistance(initialTargetPose.getTranslation());

    JSONObject result = MCPTools.successResponse();

    if (reachedTarget[0]) {
      result.put("message", "Successfully approached coral and ran intake");
      result.put("reached_target", true);
    } else if (stuckCounter[0] >= STUCK_THRESHOLD) {
      result.put("success", false);
      result.put("message", "Robot got stuck while approaching coral");
      result.put("reached_target", false);
    }

    result.put("target_x", initialTargetPose.getX());
    result.put("target_y", initialTargetPose.getY());
    result.put("initial_distance_meters", initialDistance);
    result.put("final_distance_meters", finalDistance);
    result.put("current_x", finalPose.getX());
    result.put("current_y", finalPose.getY());
    return result;
  }
}
