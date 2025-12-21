package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;
import frc.robot.utility.MotionMagicConfig;
import frc.robot.utility.PIDGains;

public class ElevatorConstants {

  public static final ElevatorConfig ELEVATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ElevatorConfig(
            CAN.at(41, "Elevator 1"), CAN.at(39, "Elevator 2"), 1 / (1.125 * Math.PI / 3));
        case SIM -> new ElevatorConfig(
            CAN.at(41, "Elevator 1"), CAN.at(39, "Elevator 2"), 1 / (1.125 * Math.PI / 3));
        default -> new ElevatorConfig(0, 0, 1.6875); // FIXME
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(
            4, 0, 0, 0, 0.1558, 0.00469, 0.0); // CHANGE VALUES WHEN CAD FINISHES
        case SIM -> new PIDGains(
            4, 0, 0, 0, 0.1558, 0.00469, 0.0); // CHANGE VALUES WHEN CAD FINISHES
        default -> new PIDGains(1.5, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(200, 200, 0);
        case SIM -> new MotionMagicConfig(200, 200, 0);
        default -> new MotionMagicConfig(0, 0, 0);
      };

  public record ElevatorConfig(int motorID, int motorID2, double reduction) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final boolean OPPOSE_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 1;

  public static final double MIN_SAFE_HEIGHT_FOR_ARM_ROTATION = 0;
  public static final double MIN_HEIGHT = 0;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 16;

  public static final double LOWER_VOLT_LIMIT = -16;

  public static final double SUPPLY_CURRENT_LIMIT = 100;

  // public static final int ZEROING_CURRENT_LIMIT = 20;

  public static final double UPPER_EXTENSION_LIMIT = 64;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -2;

  public static final double ZEROING_OFFSET = -0.30; // offset in inches

  public static final double ZEROING_VOLTAGE_THRESHOLD =
      switch (Constants.getRobotType()) {
        case SIM -> 1.5;
        default -> 15;
      };

  public static record ElevatorPhysicalConstants(
      double elevatorMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity) {}

  public static final ElevatorPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ElevatorPhysicalConstants(6.52900857, 0.0142875, 0, 5, true);
        case COMP -> new ElevatorPhysicalConstants(6.52900857, 0.0142875, 0, 0, false);
        default -> new ElevatorPhysicalConstants(6.52900857, 0.0142875, 0, 0, false);
      }; // CHANGE VALUES WHEN CAD FINISHES

  public static final Transform3d ELEVATOR_BASE_3D_OFFSET =
      switch (Constants.getRobotType()) {
        default -> new Transform3d(
            new Translation3d(
                Units.inchesToMeters(3.2575),
                Units.inchesToMeters(0),
                Units.inchesToMeters(4.9801)),
            new Rotation3d(0, 0, 0));
      };
}
