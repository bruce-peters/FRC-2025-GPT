package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.IntakeObjectCommand;
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

    // Get target position info
    Pose2d targetPose = objectDetection.getTargetPosition();
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // Schedule the IntakeObjectCommand
    Command intakeObjectCommand =
        new IntakeObjectCommand(swerve, objectDetection, intakeController);
    container.scheduleAndWait(intakeObjectCommand);

    JSONObject result = MCPTools.successResponse();
    result.put("message", "Approaching and intaking coral");
    result.put("target_x", targetPose.getX());
    result.put("target_y", targetPose.getY());
    result.put("distance_meters", distance);
    return result;
  }
}
