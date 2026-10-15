package frc.robot.subsystems.climb.climbPivot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ClimbPivotConstants {
  public static final ClimbPivotConfig CLIMB_PIVOT_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new ClimbPivotConfig(CAN.at(36, "Climb Pivot Motor"), 1, 40, -0.3224);
        case SIM -> new ClimbPivotConfig(36, 2.5, 45, 0.201);
        default -> new ClimbPivotConfig(0, 1, 0, 0d);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(1000, 0, 0, 0, 30, 0, 0.1);
        case SIM -> new PIDGains(600, 0, 0, 0, 66.5, 5.714, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0.5, 0.3, 0);
        case SIM -> new MotionMagicConfig(2, 1, 0);
        default -> new MotionMagicConfig(0, 0, 0);
      };

  public record ClimbPivotConfig(
      int motorID, double reduction, int canCoderID, double canCoderOffset) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
  public static final SensorDirectionValue CANCODER_DIRECTION =
      SensorDirectionValue.CounterClockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0.005;

  public record MotionMagicConfig(double acceleration, double cruiseVelocity, double jerk) {}

  // SOFT LIMITS
  public static final double LOWWER_EXTENSION_LIMIT = -0.04d;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double UPPER_VOLT_LIMIT_CLIMBING = 3;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final double SUPPLY_CURRENT_LIMIT = 30;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -1;
  public static final double ZEROING_OFFSET = 0; // offset in inches
  public static final double ZEROING_VOLTAGE_THRESHOLD = 4;

  // INDUCTION SENSOR
  public static final int INDUCTION_PORT_NUMBER = 6;

  // PHYSICAL CONSTANTS
  public static record ClimbPivotPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  public static final ClimbPivotPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ClimbPivotPhysicalConstants(0.006, 0.487, 0, 1.57, false);
        case COMP -> new ClimbPivotPhysicalConstants(0.006, 0.487, 0, 1.57, false);
      };
}
