package frc.robot.subsystems.objectDetection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;

public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public ObjectDetectionIOLimelight() {}

  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.xErr = Units.Degrees.of(table.getEntry("tx").getDouble(0));
    inputs.yErr = Units.Degrees.of(table.getEntry("ty").getDouble(0));
    inputs.targetArea = Units.Percent.of(table.getEntry("ta").getDouble(0));
  }
}
