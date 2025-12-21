package frc.robot.mcp.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.RobotState;
import frc.robot.mcp.MCPTool;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.superstructure.SuperstructureController;
import frc.robot.subsystems.swerve.DriveConstants;
import org.json.simple.JSONObject;

public class GetRobotStatusTool implements MCPTool {
  private final SuperstructureController superstructureController;
  private final ObjectDetection objectDetection;

  public GetRobotStatusTool(
      SuperstructureController superstructureController, ObjectDetection objectDetection) {
    this.superstructureController = superstructureController;
    this.objectDetection = objectDetection;
  }

  @Override
  public String getName() {
    return "GetRobotStatus";
  }

  @Override
  public String getDescription() {
    return "Gets the current position of the robot and state of the superstructure";
  }

  @Override
  public JSONObject getInputSchema() {
    return MCPTools.createInputSchema(new JSONObject());
  }

  @Override
  public JSONObject execute(JSONObject args) {
    // Get robot pose
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    // Get superstructure state
    SuperstructureController.SuperstructureState superstructureState =
        superstructureController.getSuperstructureState();
    SuperstructureController.SuperstructurePose currentSuperstructurePose =
        superstructureController.getCurrentSuperstructurePose();
    SuperstructureController.SuperstructurePose targetSuperstructurePose =
        superstructureController.getTargetSuperstructurePose();
    boolean atTarget = superstructureController.superstructureReachedTarget();

    // Build response
    JSONObject result = new JSONObject();

    // Robot position
    JSONObject robotPose = new JSONObject();
    robotPose.put("x", currentPose.getX());
    robotPose.put("y", currentPose.getY());
    robotPose.put("rotation_degrees", currentPose.getRotation().getDegrees());
    result.put("robot_pose", robotPose);

    // Superstructure current state
    JSONObject superstructureInfo = new JSONObject();
    superstructureInfo.put("state", superstructureState.toString());
    superstructureInfo.put("at_target", atTarget);

    JSONObject currentPoseInfo = new JSONObject();
    currentPoseInfo.put(
        "elevator_height_inches", currentSuperstructurePose.elevatorHeight.in(Units.Inches));
    currentPoseInfo.put("arm_angle_degrees", currentSuperstructurePose.armAngle.in(Units.Degrees));
    currentPoseInfo.put("arm_direction", currentSuperstructurePose.armDirection.toString());
    superstructureInfo.put("current_pose", currentPoseInfo);

    JSONObject targetPoseInfo = new JSONObject();
    targetPoseInfo.put(
        "elevator_height_inches", targetSuperstructurePose.elevatorHeight.in(Units.Inches));
    targetPoseInfo.put("arm_angle_degrees", targetSuperstructurePose.armAngle.in(Units.Degrees));
    targetPoseInfo.put("arm_direction", targetSuperstructurePose.armDirection.toString());
    superstructureInfo.put("target_pose", targetPoseInfo);

    result.put("superstructure", superstructureInfo);

    // Add reef positions and distances (for obstacle avoidance)
    JSONObject reefInfo = new JSONObject();

    // Blue reef position (constant)
    Translation2d blueReefCenter = DriveConstants.BLUE_REEF_ORIGIN;
    JSONObject blueReef = new JSONObject();
    blueReef.put("x", blueReefCenter.getX());
    blueReef.put("y", blueReefCenter.getY());
    blueReef.put("alliance", "BLUE");
    double distanceToBlueReef = currentPose.getTranslation().getDistance(blueReefCenter);
    blueReef.put("distance_meters", distanceToBlueReef);
    reefInfo.put("blue_reef", blueReef);

    // Red reef position (mirrored on field)
    Translation2d redReefCenter =
        new Translation2d(RobotState.fieldSizeX - blueReefCenter.getX(), blueReefCenter.getY());
    JSONObject redReef = new JSONObject();
    redReef.put("x", redReefCenter.getX());
    redReef.put("y", redReefCenter.getY());
    redReef.put("alliance", "RED");
    double distanceToRedReef = currentPose.getTranslation().getDistance(redReefCenter);
    redReef.put("distance_meters", distanceToRedReef);
    reefInfo.put("red_reef", redReef);

    // Add closest reef information
    if (distanceToBlueReef < distanceToRedReef) {
      reefInfo.put("closest_reef", "BLUE");
      reefInfo.put("closest_reef_distance_meters", distanceToBlueReef);
    } else {
      reefInfo.put("closest_reef", "RED");
      reefInfo.put("closest_reef_distance_meters", distanceToRedReef);
    }

    result.put("reefs", reefInfo);

    // Add object detection information (coral detection)
    JSONObject objectDetectionInfo = new JSONObject();
    boolean coralDetected = objectDetection.coralInVision();
    objectDetectionInfo.put("coral_detected", coralDetected);

    if (coralDetected) {
      Pose2d coralTargetPose = objectDetection.getTargetPosition();
      double distanceToCoralMeters =
          currentPose.getTranslation().getDistance(coralTargetPose.getTranslation());

      JSONObject coralPosition = new JSONObject();
      coralPosition.put("x", coralTargetPose.getX());
      coralPosition.put("y", coralTargetPose.getY());
      coralPosition.put("rotation_degrees", objectDetection.getTargetRotation().getDegrees());
      coralPosition.put("distance_meters", distanceToCoralMeters);

      objectDetectionInfo.put("coral_position", coralPosition);
      objectDetectionInfo.put(
          "coral_distance_x_meters", objectDetection.getCoralDistanceX().in(Units.Meters));
      objectDetectionInfo.put(
          "coral_distance_y_meters", objectDetection.getCoralDistanceY().in(Units.Meters));
      objectDetectionInfo.put(
          "angle_to_coral_degrees", objectDetection.getAngleToTarget().in(Units.Degrees));
    }

    result.put("object_detection", objectDetectionInfo);
    result.put("success", true);

    return result;
  }
}
