package frc.robot.subsystems.objectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.RobotState;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionIOSim implements ObjectDetectionIO {
  private static final double FOV_HORIZONTAL = 27.0; // degrees
  private static final double FOV_VERTICAL = 27.0; // degrees
  private static final double CORAL_DIAMETER = 0.254; // meters (10 inches)

  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = true;

    Pose3d[] corals = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Logger.recordOutput("Object Detection/Corals", corals);

    if (corals.length == 0) {
      inputs.xErr = Units.Degrees.zero();
      inputs.yErr = Units.Degrees.zero();
      inputs.targetArea = Units.Percent.zero();
      return;
    }

    // Get robot pose and camera position
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    Pose2d cameraPose = getCameraPose(robotPose);

    // Find the closest coral within FOV
    Pose3d closestCoral = null;
    double closestDistance = Double.MAX_VALUE;
    double bestTx = 0, bestTy = 0;

    for (Pose3d coral : corals) {
      Translation2d coralPosition = coral.getTranslation().toTranslation2d();

      // Transform coral position to camera-relative coordinates
      Translation2d relativePosition = coralPosition.minus(cameraPose.getTranslation());
      Translation2d rotatedPosition =
          relativePosition.rotateBy(cameraPose.getRotation().unaryMinus());

      double dx = rotatedPosition.getX();
      double dy = rotatedPosition.getY();
      // Assume coral is at height 0
      double dz = 0.0 - ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.z().in(Units.Meters);

      // Camera pitch is measured from straight down (90° from horizontal)
      // So actual pitch from horizontal = pitchAngle - 90°
      double cameraPitchFromHorizontal =
          ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.pitchAngle().in(Units.Degrees) - 90.0;

      // Calculate horizontal angle (tx)
      double horizontalAngle = Math.toDegrees(Math.atan2(dy, dx));

      // Calculate angle from camera to coral in vertical plane
      double groundDistance = Math.hypot(dx, dy);
      double angleToCoralFromHorizontal = Math.toDegrees(Math.atan2(dz, groundDistance));

      // Vertical angle (ty) relative to camera's optical axis
      double verticalAngle = angleToCoralFromHorizontal - cameraPitchFromHorizontal;

      // Check if within FOV
      if (Math.abs(horizontalAngle) <= FOV_HORIZONTAL
          && Math.abs(verticalAngle) <= FOV_VERTICAL
          && dx > 0) { // coral is in front of camera

        double totalDistance = Math.sqrt(dx * dx + dy * dy + dz * dz);

        if (totalDistance < closestDistance) {
          closestDistance = totalDistance;
          closestCoral = coral;
          bestTx = -horizontalAngle;
          bestTy = verticalAngle;
        }
      }
    }
    Logger.recordOutput("Object Detection/Closest coral", closestCoral);

    if (closestCoral != null) {
      inputs.xErr = Units.Degrees.of(bestTx);
      inputs.yErr = Units.Degrees.of(bestTy);

      // Calculate target area (larger when closer)
      double angularSize = Math.toDegrees(2 * Math.atan(CORAL_DIAMETER / (2 * closestDistance)));
      double area = (angularSize / (2 * FOV_HORIZONTAL)) * (angularSize / (2 * FOV_VERTICAL)) * 100;
      inputs.targetArea = Units.Percent.of(area);
    } else {
      inputs.xErr = Units.Degrees.zero();
      inputs.yErr = Units.Degrees.zero();
      inputs.targetArea = Units.Percent.zero();
    }
  }

  private Pose2d getCameraPose(Pose2d robotPose) {
    // Map constants: y (forward/backward) -> WPILib X, x (left/right) -> WPILib Y
    // Camera is facing backward, so add 180° to the rotation
    Transform2d cameraTransform =
        new Transform2d(
            new Translation2d(
                ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                    .y()
                    .in(Units.Meters), // forward/backward -> X
                ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS
                    .x()
                    .in(Units.Meters) // left/right -> Y
                ),
            Rotation2d.fromDegrees(
                ObjectDetectionConstants.CAMERA_POSITION_CONSTANTS.yawAngle().in(Units.Degrees)));
    return robotPose.plus(cameraTransform);
  }
}
