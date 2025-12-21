package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
  private ObjectDetectionIO objectDetectionIO;
  private final ObjectDetectionIOInputsAutoLogged inputs;
  // make a velocity algorithm (maybe)

  public ObjectDetection(ObjectDetectionIO objectDetectionIO) {
    this.objectDetectionIO = objectDetectionIO;
    inputs = new ObjectDetectionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    objectDetectionIO.updateInputs(inputs);
    Logger.processInputs("Object Detection", inputs);

    Logger.recordOutput("Object Detection/Error Horizontal", inputs.xErr);
    Logger.recordOutput("Object Detection/Error Vertical", inputs.yErr);
    Logger.recordOutput("Object Detection/Dy", getCoralDistanceY());
    Logger.recordOutput("Object Detection/Dx", getCoralDistanceX());
    Logger.recordOutput("Object Detection/Target Heading", getTargetRotation());
    Logger.recordOutput("Object Detection/Angle to target", getAngleToTarget());
    Logger.recordOutput("Object Detection/Target position", getTargetPosition());
    Logger.recordOutput("Object Detection/Detected coral", coralInVision());
  }

  /** whether or not we currently see a coral */
  public boolean coralInVision() {
    return inputs.targetArea.compareTo(ObjectDetectionConstants.TARGET_AREA_THRESHOLD) >= 0;
  }

  /**
   * Gets the target heading for the robot in order to make it go straight toward the object
   *
   * @return
   */
  public Rotation2d getTargetRotation() {
    if (!coralInVision()) { // if we don't see a coral, just return the current robot heading
      return RobotState.getInstance().getEstimatedPose().getRotation();
    }

    Angle angleToTarget = getAngleToTarget();

    return RobotState.getInstance()
        .getEstimatedPose()
        .getRotation()
        .minus(new Rotation2d(angleToTarget));
  }

  public Pose2d getTargetPosition() {
    if (!coralInVision()) {
      return RobotState.getInstance().getEstimatedPose();
    }

    Translation2d translation =
        new Translation2d(getCoralDistanceY(), getCoralDistanceX().times(-1));

    Transform2d targetPoseRelative = new Transform2d(translation, new Rotation2d());
    Logger.recordOutput("Object Detect/Target position relative", targetPoseRelative);

    Pose2d targetPose = RobotState.getInstance().getEstimatedPose().plus(targetPoseRelative);
    Logger.recordOutput("Object Detect/Target pose intermediary", targetPose);

    targetPose = new Pose2d(targetPose.getTranslation(), getTargetRotation());

    return targetPose;
  }

  public Angle getAngleToTarget() {

    double distanceY = getCoralDistanceY().in(Units.Meters);
    double distanceX = getCoralDistanceX().in(Units.Meters);

    // arctan(t_x / t_y)
    Angle angleToTarget = Units.Radians.of(Math.atan2(distanceX, distanceY)); // radians
    return angleToTarget;
  }

  public Distance getCoralDistanceY() {
    Angle targetPitch = inputs.yErr;
    Angle targetYaw = inputs.xErr;

    double distanceY =
        Math.cos(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                        .yawAngle()
                        .plus(targetYaw)
                        .in(Units.Radians))
                * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                    * Math.tan(
                        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                            .pitchAngle()
                            .plus(targetPitch)
                            .in(Units.Radians))
                    / Math.cos(targetYaw.in(Units.Radians)))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.y().in(Units.Meters);
    return Units.Meters.of(distanceY);
  }

  public Distance getCoralDistanceX() {
    Angle targetPitch = inputs.yErr;
    Angle targetYaw = inputs.xErr;

    double distanceX =
        (Math.sin(
                    ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                        .yawAngle()
                        .plus(targetYaw)
                        .in(Units.Radians))
                * (ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters)
                    * Math.tan(
                        ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                            .pitchAngle()
                            .plus(targetPitch)
                            .in(Units.Radians))
                    / Math.cos(targetYaw.in(Units.Radians)))
            - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.x().in(Units.Meters));
    return Units.Meters.of(distanceX);
  }
}
