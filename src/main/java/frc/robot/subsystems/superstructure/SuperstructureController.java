package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.Arm.ArmTarget;
import frc.robot.subsystems.superstructure.arm.ArmConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.Elevator.ElevatorTarget;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class SuperstructureController extends SubsystemBase {

  /**
   * Enum for the different states of the superstructure each state represents a different position
   * or configuration of the superstructure (Arm and Elevator)
   */
  public enum SuperstructureState {
    /** The stow mode (elevator down, arm up) */
    STOW(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.BOTTOM, ArmTarget.TOP, ArmDirection.BOTH)),
    /** The highest point it can go */
    TOP(SuperstructurePose.fromTargetStates(ElevatorTarget.TOP, ArmTarget.TOP, ArmDirection.BOTH)),
    /** Picking up L2 algae from the reef */
    L2_ALGAE(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.ALGAE_INTAKE_REEF_L2, ArmTarget.STRAIGHT, ArmDirection.BOTH)),
    /** Picking up L2 algae from the reef */
    L3_ALGAE(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.ALGAE_INTAKE_REEF_L3, ArmTarget.STRAIGHT, ArmDirection.BOTH)),
    /** Picking up algae from the ground */
    GROUND_ALGAE(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.GROUND_ALGAE, ArmTarget.GROUND_ALGAE, ArmDirection.BOTH)),
    BARGE_RIGHT(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.TOP, ArmTarget.BARGE_RIGHT, ArmDirection.BOTH)),
    CLIMB(
        SuperstructurePose.fromTargetStates(
            ElevatorTarget.CLIMB, ArmTarget.CLIMB, ArmDirection.BOTH)),
    /** Zeroing the subystem -- elevator down, pivot up (much like stow) */
    ZEROING(new SuperstructurePose(Units.Inches.of(0), Units.Degrees.of(90), ArmDirection.BOTH));

    private final SuperstructurePose targetPose;

    /**
     * Gets the target pose for the superstructure state The arm direction part of the target pose
     * tells future logic weather it matters (CLOCKWISE, COUNTERCLOCKWISE) or doesn't (BOTH)
     *
     * @return the target pose for this superstructure state
     */
    public SuperstructurePose getTargetPose() {
      return targetPose;
    }

    private SuperstructureState(SuperstructurePose targetPose) {
      this.targetPose = targetPose;
    }
  }

  /** Enum for the direction the arm should move when going to a position */
  public enum ArmDirection {
    /** Clockwise when looking at the mechanism from the intake side */
    CLOCKWISE,
    /** Counter Clockwise when looking at the mechanism from the intake side */
    COUNTERCLOCKWISE,
    /**
     * The Arm can move in either direction and chooses the most optimal path Basically hands off
     * control to future logic
     */
    BOTH
  }

  /** Class for storing pose information from the superstructure */
  public static class SuperstructurePose {
    /** Height of the elevator in meters */
    public final Distance elevatorHeight;
    /**
     * Angle of the arm relative to the horizon (horizontal right when looking from intake side is 0
     * degrees)
     */
    public final Angle armAngle;
    /** Direction the arm should move when going to a position */
    public final ArmDirection armDirection;

    /**
     * Constructor for the superstructure pose
     *
     * @param elevatorHeight height of the elevator in meters
     * @param armAngle angle of the arm in degrees
     * @param armDirection direction the arm should move when going to a position
     */
    public SuperstructurePose(Distance elevatorHeight, Angle armAngle, ArmDirection armDirection) {
      this.elevatorHeight = elevatorHeight;
      this.armAngle = armAngle;
      this.armDirection = armDirection;
    }

    public LoggedMechanism2d getAsMechanism2d() {
      LoggedMechanism2d mech =
          new LoggedMechanism2d(
              Units.Inches.of(50).in(Units.Meters), Units.Inches.of(50).in(Units.Meters));
      mech.getRoot("Superstructure", Units.Inches.of(25).in(Units.Meters), 0)
          .append(new LoggedMechanismLigament2d("Elevator", elevatorHeight.in(Units.Meters), 90))
          .append(
              new LoggedMechanismLigament2d(
                  "Arm",
                  Units.Inches.of(ArmConstants.ARM_LENGTH).in(Units.Meters),
                  armAngle.in(Units.Degrees) - 90));
      return mech;
    }

    /** Get the Pose3d of the elevator based on the current elevator height */
    public Pose3d getElevatorPose3d() {
      return new Pose3d()
          .plus(
              ElevatorConstants.ELEVATOR_BASE_3D_OFFSET.plus(
                  new Transform3d(
                      new Translation3d(
                          0,
                          0,
                          elevatorHeight.in(Units.Meters)), // Add the current elevator's extension
                      new Rotation3d(0, 0, 0)))); // The elevator doesn't rotate, duh
    }

    /** Get the Pose3d of the arm pivot based on the current elevator height and arm */
    public Pose3d getPivotPose3d() {
      return this.getElevatorPose3d()
          .plus(ArmConstants.ELEVATOR_TO_ARM_TRANSFORM3D)
          .plus(
              new Transform3d(
                  new Translation3d(0, 0, 0),
                  new Rotation3d(armAngle.minus(Units.Degrees.of(90)).in(Units.Radians), 0, 0)));
    }

    /**
     * Constructs a SuperstructurePose from Elevator and Arm targets
     *
     * @param elevatorTarget
     * @param armTarget
     * @param armDirection
     * @return
     */
    public static SuperstructurePose fromTargetStates(
        ElevatorTarget elevatorTarget, ArmTarget armTarget, ArmDirection armDirection) {
      return new SuperstructurePose(
          Units.Inches.of(elevatorTarget.getPosition()),
          Units.Degrees.of(armTarget.getPosition()),
          armDirection);
    }
  }

  /**
   * Record for the physical constraints of the superstructure
   *
   * @param minElevatorHeight minimum height of the elevator in meters
   * @param maxElevatorHeight maximum height of the elevator in meters
   * @param minArmAngle minimum angle of the arm in degrees -- centered around 0/360 being directly
   *     right when looking from the intake side
   * @param maxArmAngle maximum angle of the arm in degrees -- centered around 0/360 being directly
   *     right when looking from the intake side
   */
  record SuperstructureConstraints(
      Distance minElevatorHeight,
      Distance maxElevatorHeight,
      Angle minArmAngle,
      Angle maxArmAngle) {}
  ;

  /** The current target state of the superstructure */
  private SuperstructureState superstructureState = SuperstructureState.STOW;

  // private SuperstructureState savedState = superstructureState;

  /**
   * Get the current target state of the superstructure
   *
   * @return The current target state of the superstructure
   */
  public SuperstructureState getSuperstructureState() {
    return superstructureState;
  }

  /**
   * Wether or not the superstructure has reached its target state
   *
   * @return true if the superstructure has reached its target state
   */
  public boolean superstructureReachedTargetState() {
    return arm.reachedTarget() && elevator.reachedTarget();
  }

  /**
   * Set the current target state of the superstructure
   *
   * @param state
   */
  public void setSuperstructureState(SuperstructureState state) {
    // if (superstructureState == SuperstructureState.BARGE_RIGHT) {
    //   this.superstructureState = SuperstructureState.TOP;
    //   savedState = state;
    // } else if (state == SuperstructureState.BARGE_RIGHT) {
    //   this.superstructureState = SuperstructureState.TOP;
    //   savedState = state;
    // } else {
    this.superstructureState = state;
    // }
  }

  public Command setTargetSuperstructureState(SuperstructureState state) {
    return new InstantCommand(
            () -> {
              this.superstructureState = state;
            },
            this)
        .withTimeout(.02)
        .andThen(new WaitUntilCommand(this::superstructureReachedTarget));
  }

  public boolean superstructureReachedTarget() {
    return elevator.reachedTarget() && arm.reachedTarget();
  }

  /**
   * Returns a command the sets the superstructure state and waits until the superstructure reaches
   * the target state
   *
   * @param state
   */
  public Command setSuperstructureStateCommand(SuperstructureState state) {
    return new InstantCommand(() -> setSuperstructureState(state), this)
        .andThen(
            new WaitCommand(.02)
                .andThen(new WaitUntilCommand(this::superstructureReachedTargetState)));
  }

  /**
   * Gets the relative angle to give to the motor controller to reach the given absolute angle
   *
   * @param absoluteAngle (0-360 degrees) centered at directly right when looking from the intake
   *     side
   * @param armDirection the direction the arm should move when going to the position
   * @return the relative angle to give to the motor controller
   */
  public double absoluteToRelativeTarget(
      Angle absoluteAngle, SuperstructurePose currentPose, ArmDirection armDirection) {

    // Get current raw encoder position (can be any value)
    double currentRawPosition = currentPose.armAngle.in(Units.Degrees);

    // Get the current absolute angle (0-360)
    double currentAbsoluteAngle = normalizeAngle(currentRawPosition);

    // Get the target absolute angle (0-360)
    double targetAbsoluteAngle = normalizeAngle(absoluteAngle.in(Units.Degrees));

    // Normalize delta to [-180, 180] range
    double deltaAngle = calculateShortestDeltaAngle(targetAbsoluteAngle, currentAbsoluteAngle);

    // Calculate the two possible targets
    double closestAngle = currentRawPosition + deltaAngle;

    double farthestAngle = currentRawPosition + deltaAngle + (deltaAngle > 0 ? -360.0 : 360.0);

    // Choose target based on direction preference
    return switch (armDirection) {
      case CLOCKWISE -> deltaAngle >= 0 ? farthestAngle : closestAngle;
      case COUNTERCLOCKWISE -> deltaAngle <= 0 ? farthestAngle : closestAngle;
      case BOTH -> closestAngle; // Use the shortest path
    };
  }

  /**
   * Normalizes an angle to be between 0-360 degrees
   *
   * @param angle The angle to normalize
   * @return The normalized angle between 0-360 degrees
   */
  private static double normalizeAngle(double angle) {
    return ((angle % 360.0) + 360.0) % 360.0;
  }

  /**
   * Calculates the shortest delta between two angles
   *
   * @param target Target angle (0-360)
   * @param current Current angle (0-360)
   * @return Delta angle in range [-180, 180]
   */
  private static double calculateShortestDeltaAngle(double target, double current) {
    // Normalize both angles to [0, 360) range first
    target = normalizeAngle(target);
    current = normalizeAngle(current);

    // Calculate the direct difference
    double deltaAngle = target - current;

    // Normalize to [-180, 180] range for shortest path
    if (deltaAngle > 180.0) {
      deltaAngle -= 360.0;
    } else if (deltaAngle < -180.0) {
      deltaAngle += 360.0;
    }

    return deltaAngle;
  }

  // subsystems to control
  private Elevator elevator;
  private Arm arm;

  /**
   * Constructor for the superstructure controller
   *
   * @param elevator the elevator subsystem to control
   * @param arm the arm subsystem to control
   */
  public SuperstructureController(Elevator elevator, Arm arm) {
    // setup the subsystems
    this.elevator = elevator;
    this.arm = arm;

    // set the initial target state
    setSuperstructureState(SuperstructureState.ZEROING);
  }

  @Override
  public void periodic() {
    // Run state logic and update targets
    updateTargets();

    // Update subsystem periodics
    updateSubsystemPeriodics();

    // Log data
    logData();
  }

  /** Updates targets for subsystems based on current state and constraints */
  private void updateTargets() {
    SuperstructurePose targetPose = getTargetSuperstructurePose();
    SuperstructurePose currentPose = getCurrentSuperstructurePose();
    // handle elevator zeroing
    if (superstructureState == SuperstructureState.ZEROING) {
      elevator.setZeroing(true);
      if (elevator.getFilteredSupplyCurrentAmps() > ElevatorConstants.ZEROING_VOLTAGE_THRESHOLD) {
        elevator.setOffset();
        elevator.setZeroing(false);
        superstructureState = SuperstructureState.STOW; // put us in stow state by default
      }
    } else {
      elevator.setPositionTargetManual(targetPose.elevatorHeight.in(Units.Inches));
      elevator.setZeroing(false);
    }
    arm.setPositionTargetManual(
        absoluteToRelativeTarget(targetPose.armAngle, currentPose, targetPose.armDirection));
  }

  /** Updates subsystem periodic methods */
  private void updateSubsystemPeriodics() {
    elevator.periodic();
    arm.periodic();
  }

  /** Logs relevant data about the superstructure */
  private void logData() {
    SuperstructurePose targetPose = getTargetSuperstructurePose();
    SuperstructurePose currentPose = getCurrentSuperstructurePose();
    SuperstructureConstraints constraints = getSuperstructureConstraints();

    // Log state
    Logger.recordOutput("Superstructure/SuperstructureState", superstructureState);

    // Log target pose data
    Logger.recordOutput("Superstructure/TargetPose/Mechanism2d", targetPose.getAsMechanism2d());
    Logger.recordOutput(
        "Superstructure/TargetPose/ElevatorHeight", targetPose.elevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/TargetPose/ArmAngle", targetPose.armAngle.in(Units.Degrees));
    Logger.recordOutput("Superstructure/TargetPose/ArmDirection", targetPose.armDirection);

    // Log current pose data
    Logger.recordOutput("Superstructure/CurrentPose/Mechanism2d", currentPose.getAsMechanism2d());
    Logger.recordOutput(
        "Superstructure/CurrentPose/ElevatorHeight", currentPose.elevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/CurrentPose/ArmAngle", currentPose.armAngle.in(Units.Degrees));
    Logger.recordOutput("Superstructure/CurrentPose/ArmDirection", currentPose.armDirection);
    Logger.recordOutput("Superstructure/CurrentPose/ElevatorPose", currentPose.getElevatorPose3d());
    Logger.recordOutput("Superstructure/CurrentPose/PivotPose", currentPose.getPivotPose3d());

    // Log constraints
    Logger.recordOutput(
        "Superstructure/Constraints/MaxElevator", constraints.maxElevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/Constraints/MinElevator", constraints.minElevatorHeight.in(Units.Inches));
    Logger.recordOutput(
        "Superstructure/Constraints/MaxArm", constraints.maxArmAngle.in(Units.Degrees));
    Logger.recordOutput(
        "Superstructure/Constraints/MinArm", constraints.minArmAngle.in(Units.Degrees));
  }

  /**
   * Get the current pose of the superstructure based on the readings from the subsystems
   *
   * @return The current superstructure pose
   */
  public SuperstructurePose getCurrentSuperstructurePose() {
    Distance currentElevatorHeight = Units.Inches.of(elevator.getPosition());
    Angle currentArmAngle = Units.Degrees.of(arm.getPosition());
    ArmDirection currentArmDirection = ArmDirection.BOTH; // TODO: figure out a way to get this
    return new SuperstructurePose(currentElevatorHeight, currentArmAngle, currentArmDirection);
  }

  /**
   * Get the target pose for the current target state
   *
   * @return The modified superstructure target pose based on the current state and the physical
   *     constraints of the mechanism
   */
  public SuperstructurePose getTargetSuperstructurePose() {
    // Get the prerequisite info for these calculations
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    // 1. Calculate the Min and Max heights and angles for the elevator and pivot
    SuperstructureConstraints constraints = getSuperstructureConstraints();
    Angle normalizedCurrentArmAngle =
        Units.Degrees.of(normalizeAngle(currentPose.armAngle.in(Units.Degrees)));
    Angle normalizedTargetArmAngle =
        Units.Degrees.of(normalizeAngle(superstructureState.targetPose.armAngle.in(Units.Degrees)));

    // 2. Clamp the elevator target height between the min and max
    Distance targetElevatorHeight =
        clamp(
            superstructureState.getTargetPose().elevatorHeight,
            constraints.minElevatorHeight,
            constraints.maxElevatorHeight);

    // 3. Figure out what direction the arm should be allowed to move
    ArmDirection targetArmDirection = superstructureState.getTargetPose().armDirection;
    double shortestDeltaAngleToTarget =
        calculateShortestDeltaAngle(
            normalizedTargetArmAngle.in(Units.Degrees),
            normalizedCurrentArmAngle.in(Units.Degrees));
    if (targetArmDirection == ArmDirection.BOTH) {
      // see what direction is the most optimal direction and set our direction based
      // on that
      targetArmDirection =
          (shortestDeltaAngleToTarget >= 0)
              ? ArmDirection.COUNTERCLOCKWISE
              : ArmDirection.CLOCKWISE;
    }

    // 4. Figure out if we need to change the elevator height to allow for
    // pivot rotation
    if (armGoesThroughBottom(
        normalizedCurrentArmAngle, normalizedTargetArmAngle, targetArmDirection)) {
      targetElevatorHeight =
          Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION)
              .plus(Units.Inches.of(ArmConstants.ARM_LENGTH))
              .plus(Units.Inches.of(2.0)); // TODO: make the 2.0 an actual
      // constant value
    }

    // 5. Smart clamp our arm target angle using constraints and target arm
    // direction
    Angle targetArmAngle =
        smartClampArmTargetAngle(
            normalizedCurrentArmAngle, normalizedTargetArmAngle, targetArmDirection, constraints);

    // 6. Figure out what final arm direction to go in to get to that target
    double finalDeltaAngle =
        calculateShortestDeltaAngle(
            targetArmAngle.in(Units.Degrees), normalizedCurrentArmAngle.in(Units.Degrees));
    ArmDirection finalArmDirection;
    if (finalDeltaAngle > 0) {
      finalArmDirection = ArmDirection.COUNTERCLOCKWISE;
    } else if (finalDeltaAngle < 0) {
      finalArmDirection = ArmDirection.CLOCKWISE;
    } else {
      finalArmDirection = ArmDirection.BOTH; // we are already at the target
    }

    return new SuperstructurePose(targetElevatorHeight, targetArmAngle, finalArmDirection);
  }

  /**
   * Uses angle logic and the direction we want our arm to move in to clamp the target angle between
   * our min and max angles
   *
   * @param currentAngle the angle that our superstructure is currently at
   * @param targetAngle
   * @param targetArmDirection
   * @param constraints
   * @return
   */
  public static Angle smartClampArmTargetAngle(
      Angle currentAngle,
      Angle targetAngle,
      ArmDirection targetArmDirection,
      SuperstructureConstraints constraints) {

    // creating some variables that will get used later
    Angle normalizedCurrentAngle = Units.Degrees.of(normalizeAngle(currentAngle.in(Units.Degrees)));
    Angle normalizedTargetAngle = Units.Degrees.of(normalizeAngle(targetAngle.in(Units.Degrees)));
    double shortestDeltaAngleToTarget =
        calculateShortestDeltaAngle(
            normalizeAngle(targetAngle.in(Units.Degrees)),
            normalizeAngle(normalizedCurrentAngle.in(Units.Degrees)));
    Angle armMinimumAngle =
        Units.Degrees.of(normalizeAngle(constraints.minArmAngle.in(Units.Degrees)));
    Angle armMaximumAngle =
        Units.Degrees.of(normalizeAngle(constraints.maxArmAngle.in(Units.Degrees)));

    // 1. Modify the target arm pose based on the direction we want to go (if we
    // want to go clockwise we go to the nearest mod of the target angle in the
    // positive direction)
    Angle modifiedTargetAngle = normalizedTargetAngle;
    if (targetArmDirection == ArmDirection.CLOCKWISE) {
      // if we are going clockwise, take the shortest delta angle and make it negative
      // and add it to the current angle
      double deltaAngle =
          (shortestDeltaAngleToTarget <= 0)
              ? shortestDeltaAngleToTarget
              : shortestDeltaAngleToTarget - 360.0;
      modifiedTargetAngle = Units.Degrees.of(normalizedCurrentAngle.in(Units.Degrees) + deltaAngle);
    } else if (targetArmDirection == ArmDirection.COUNTERCLOCKWISE) {
      // if we are going counterclockwise, take the shortest delta angle and
      // make it positive and add it to the current angle
      double deltaAngle =
          (shortestDeltaAngleToTarget >= 0)
              ? shortestDeltaAngleToTarget
              : shortestDeltaAngleToTarget + 360.0;
      modifiedTargetAngle = Units.Degrees.of(normalizedCurrentAngle.in(Units.Degrees) + deltaAngle);
    }

    // EDGE CASE 1: if our min and max angles give us almost a full revolution we
    // should just return our modified target angle
    if (Math.abs(
            calculateShortestDeltaAngle(
                armMaximumAngle.in(Units.Degrees), armMinimumAngle.in(Units.Degrees)))
        < 1.0) {
      return modifiedTargetAngle;
    }

    // EDGE CASE 2: if our arm is between our min and max on the bottom (ei. the arm
    // is at like 270 and the min is 280 and the max is 290) we should just make our
    // target position the closest min or max angle
    if (armMinimumAngle.gt(normalizedCurrentAngle) && armMaximumAngle.lt(normalizedCurrentAngle)) {
      Angle closestConstrainingAngle =
          (Math.abs(armMinimumAngle.minus(normalizedCurrentAngle).in(Units.Degrees))
                  < (Math.abs(normalizedCurrentAngle.minus(armMaximumAngle).in(Units.Degrees))))
              ? armMinimumAngle
              : armMaximumAngle; // TODO: add some logging error here
      if (armGoesThroughBottom(normalizedCurrentAngle, modifiedTargetAngle, targetArmDirection)) {
        return closestConstrainingAngle; // if we would go through the bottom, we should go to our
        // closest constraining
        // angle
      } else {
        return modifiedTargetAngle; // if we aren't going through the bottom, have at it and go to
        // our target angle
      }
    }

    // 2. Make sure that the min and max angles are actually less than and greater
    // than the current angle
    if (armMinimumAngle.gt(normalizedCurrentAngle)) {
      // make sure the minimum angle is less than the current angle
      armMinimumAngle = armMinimumAngle.minus(Units.Degrees.of(360.0));
    }
    if (armMaximumAngle.lt(normalizedCurrentAngle)) {
      // make sure the maximum angle is greater than the current angle
      armMaximumAngle = armMaximumAngle.plus(Units.Degrees.of(360.0));
    }

    // 3. Clamp the arm target angle between the min and max and return that value
    return clamp(modifiedTargetAngle, armMinimumAngle, armMaximumAngle);
  }

  /**
   * Clamp a measure between a min and max
   *
   * @param <U> the unit used in the clamping
   * @param <M> the mesure type used in the clamping
   * @param val the value to clamp
   * @param min the minimum value
   * @param max the maximum value
   * @return the clamped value
   */
  public static <U extends Unit, M extends Measure<U>> M clamp(M val, M min, M max) {
    if (val.lt(min)) {
      return min;
    } else if (val.gt(max)) {
      return max;
    } else {
      return val;
    }
  }

  /**
   * Determine if the arm will go through the bottom (270 degrees) when moving from the current
   * state to the target state in the given arm direction
   *
   * @param current
   * @param target
   * @param armDirection if set to BOTH the function will assume it will take the most optimized
   *     path
   * @return weather or not the mechanism will go through 270 on the route between the current and
   *     the target
   */
  public static boolean armGoesThroughBottom(
      Angle current, Angle target, ArmDirection armDirection) {
    double currentToTargetDelta =
        calculateShortestDeltaAngle(
            normalizeAngle(target.in(Units.Degrees)), normalizeAngle(current.in(Units.Degrees)));
    double bottomToTargetDelta =
        calculateShortestDeltaAngle(normalizeAngle(target.in(Units.Degrees)), 270.0);

    boolean armGoesThroughBottomOnOptimizedPath =
        (Math.abs(currentToTargetDelta) > Math.abs(bottomToTargetDelta)
            && currentToTargetDelta * bottomToTargetDelta >= 0);

    // figure out if were trying to go through the optimized path or not
    ArmDirection optimizedDirection =
        (currentToTargetDelta >= 0) ? ArmDirection.COUNTERCLOCKWISE : ArmDirection.CLOCKWISE;

    boolean armGoingThroughOptimizedPath =
        (armDirection == ArmDirection.BOTH || armDirection == optimizedDirection);

    if (armGoingThroughOptimizedPath) {
      return armGoesThroughBottomOnOptimizedPath;
    } else {
      return !armGoesThroughBottomOnOptimizedPath;
    }
  }

  /**
   * Get the physical constraints of the superstructure Based on where the arm and elevator
   * currently are
   *
   * @return The physical constraints of the superstructure
   */
  public SuperstructureConstraints getSuperstructureConstraints() {
    // HACK: TEMPORARILY RETURNS NO CONSTRAINTS FOR MADTOWN -- THIS COMPLETELY DISABLES THE GOOD
    // COMPLEX LOGIC
    Distance minElevatorHeight = Units.Inches.of(0);
    // Distance minElevatorHeight = getMinElevatorHeight();
    Distance maxElevatorHeight = Units.Inches.of(ElevatorConstants.UPPER_EXTENSION_LIMIT);
    Pair<Angle, Angle> armAngleConstraints =
        new Pair<>(Units.Degrees.of(-90), Units.Degrees.of(270));
    // Pair<Angle, Angle> armAngleConstraints = getArmAngleConstraints();
    Angle minArmAngle = armAngleConstraints.getFirst();
    Angle maxArmAngle = armAngleConstraints.getSecond();

    return new SuperstructureConstraints(
        minElevatorHeight, maxElevatorHeight, minArmAngle, maxArmAngle);
  }

  /**
   * Get the minimum height the elevator can be at based on the current position of the arm
   *
   * @return The minimum height the elevator can be at
   */
  private Distance getMinElevatorHeight() {
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    Distance armHeightRelativeToElevator =
        Units.Inches.of( // convert the arm length to inches
            ArmConstants.ARM_LENGTH
                * Math.sin(currentPose.armAngle.in(Units.Radians))); // vertical component

    if (armHeightRelativeToElevator.compareTo(Units.Inches.of(0)) > 0) {
      // arm is above the elevator
      return Units.Inches.of(
          ElevatorConstants
              .MIN_HEIGHT); // return the min height of the mech because we know the arm is safe
    }

    // arm is below the elevator
    Distance minSafeHeight =
        Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION)
            .plus(
                armHeightRelativeToElevator
                    .unaryMinus() // get the inverse of the arm height relative to the elevator
                ); // add that to the min safe height for the mech and we get the min safe height
    // for the elevator

    return minSafeHeight;
  }

  /**
   * Get the minimum and maximum angles the arm can be at based on the current height of the
   * elevator
   *
   * @return A pair containing the minimum and maximum angles the arm can be at
   */
  private Pair<Angle, Angle> getArmAngleConstraints() {
    SuperstructurePose currentPose = getCurrentSuperstructurePose();

    // Calculate available height above minimum safe height
    Distance heightOnElevator = calculateAvailableElevatorHeight(currentPose);
    Distance armLength = Units.Inches.of(ArmConstants.ARM_LENGTH);

    // If elevator is high enough, allow full range of motion
    if (heightOnElevator.gt(armLength)) {
      return new Pair<>(Units.Degrees.of(-90), Units.Degrees.of(270));
    }

    // Calculate angle constraints based on available height
    return calculateAngleConstraints(heightOnElevator, armLength);
  }

  /** Calculates the available height on the elevator for arm movement */
  private Distance calculateAvailableElevatorHeight(SuperstructurePose pose) {
    return pose.elevatorHeight.minus(
        Units.Inches.of(ElevatorConstants.MIN_SAFE_HEIGHT_FOR_ARM_ROTATION));
  }

  /** Calculates angle constraints based on available height and arm length */
  private Pair<Angle, Angle> calculateAngleConstraints(
      Distance heightOnElevator, Distance armLength) {
    // Calculate the minimum angle using arccos
    double angleRad = Math.acos(heightOnElevator.div(armLength).in(Units.Value));

    Angle minAngle = Units.Radians.of(angleRad);
    Angle maxAngle = Units.Radians.of(2 * Math.PI - angleRad);

    // Convert from bottom-relative to right-relative coordinates
    minAngle = minAngle.minus(Units.Degrees.of(90));
    maxAngle = maxAngle.minus(Units.Degrees.of(90));

    return new Pair<>(minAngle, maxAngle);
  }
}
