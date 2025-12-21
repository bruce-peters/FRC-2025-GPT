package frc.robot.subsystems.objectDetection;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public Angle xErr = Units.Degrees.of(0);
    public Angle yErr = Units.Degrees.of(0);
    public Dimensionless targetArea = Units.Percent.of(0);
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
