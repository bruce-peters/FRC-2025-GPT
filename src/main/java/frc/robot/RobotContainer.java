// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.ApproachReef.LevelOffsets;
import frc.robot.commands.IntakeObjectCommand;
import frc.robot.commands.ScoreL1Command;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.mcp.MCPServer;
import frc.robot.mcp.MCPToolSchemas;
import frc.robot.mcp.MCPTools;
import frc.robot.subsystems.canWatchdog.CANWatchdog;
import frc.robot.subsystems.canWatchdog.CANWatchdogIO;
import frc.robot.subsystems.claw.ClawRollers;
import frc.robot.subsystems.claw.ClawRollers.ClawRollersTarget;
import frc.robot.subsystems.claw.ClawRollersController;
import frc.robot.subsystems.claw.ClawRollersController.ClawState;
import frc.robot.subsystems.claw.ClawRollersIO;
import frc.robot.subsystems.claw.ClawRollersIOSim;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.climb.climbPivot.ClimbPivot;
import frc.robot.subsystems.climb.climbPivot.ClimbPivotIO;
import frc.robot.subsystems.climb.climbPivot.ClimbPivotIOSim;
import frc.robot.subsystems.climb.climbRollers.ClimbRollers;
import frc.robot.subsystems.climb.climbRollers.ClimbRollersIO;
import frc.robot.subsystems.climb.climbRollers.ClimbRollersIOSim;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensorIOBeambreak;
import frc.robot.subsystems.climb.climb_sensors.ClimbSensors;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIO;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.intake_pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIO;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOSim;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollersIOTalonFX;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIO;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIOCANRange;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorIOSim;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensors;
import frc.robot.subsystems.intake.intake_sensors.IntakeSensorsConstants;
import frc.robot.subsystems.l1_pivot.L1Pivot;
import frc.robot.subsystems.l1_pivot.L1PivotController;
import frc.robot.subsystems.l1_pivot.L1PivotIO;
import frc.robot.subsystems.l1_pivot.L1PivotIOSim;
import frc.robot.subsystems.l1_pivot.L1PivotIOTalonFX;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.objectDetection.ObjectDetectionIOLimelight;
import frc.robot.subsystems.objectDetection.ObjectDetectionIOSim;
import frc.robot.subsystems.rgb.RGB;
import frc.robot.subsystems.rgb.RGBIO;
import frc.robot.subsystems.superstructure.SuperstructureController;
import frc.robot.subsystems.superstructure.SuperstructureController.SuperstructureState;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.arm.ArmIO;
import frc.robot.subsystems.superstructure.arm.ArmIOSim;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFXReal;
import frc.robot.subsystems.swerve.ModuleIOTalonFXSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonvisionSim;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // DO NOT DELETE -- this actually does something important
  private RobotState robotState = RobotState.getInstance();

  // private SendableChooser<Command> autoChooser;
  private LoggedDashboardChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  @AutoLogOutput(key = "CommandedOffset")
  private LevelOffsets levelOffsets = LevelOffsets.L1_OFFSET;

  private boolean eject = false;

  private Drive swerve;
  private Vision vision;
  private RGB rgb;
  private CANWatchdog canWatchdog;
  private ObjectDetection objectDetection;
  private IntakeRollers intakeRollers;
  private IntakePivot intakePivot;
  private IntakeController intakeController;
  private IntakeSensors intakeSensors;
  private ClawRollers clawRollers;
  private ClawRollersController clawRollersController;

  private SuperstructureController superstructureController;
  private Arm arm;
  private Elevator elevator;
  private L1PivotController l1PivotController;
  private L1Pivot l1Pivot;

  private ClimbController climbController;
  private ClimbPivot climbPivot;
  private ClimbRollers climbRollers;
  private ClimbSensors climbSensors;

  public RobotContainer() {
    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFXReal(DriveConstants.MODULE_CONFIGS[3]));
          // vision =
          //     new Vision(
          //         new VisionIOPhotonvision("arducam-5", 1),
          //         new VisionIOPhotonvision("arducam-4", 2));
          // canWatchdog = new CANWatchdog(new CANWatchdogIOComp(), rgb);
          l1Pivot = new L1Pivot(new L1PivotIOTalonFX());
          intakeRollers = new IntakeRollers(new IntakeRollersIOTalonFX());
          intakePivot = new IntakePivot(new IntakePivotIOTalonFX());

          intakeSensors =
              new IntakeSensors(
                  new IntakeSensorIOCANRange(IntakeSensorsConstants.PORT_ID_1),
                  new IntakeSensorIOCANRange(IntakeSensorsConstants.PORT_ID_2));

          // elevator = new Elevator(new ElevatorIOTalonFX());
          // arm = new Arm(new ArmIOTalonFX());
          // clawRollers = new ClawRollers(new ClawRollersIOTalonFX());
          // climbPivot = new ClimbPivot(new ClimbPivotIOTalonFX());
          // climbRollers = new ClimbRollers(new ClimbRollersIOTalonFX());
          // climbSensors = new ClimbSensors(new ClimbSensorIOBeambreak());
          objectDetection = new ObjectDetection(new ObjectDetectionIOLimelight());
        }
        case SIM -> {
          SwerveDriveSimulation driveSimulation = RobotSimState.getInstance().getDriveSimulation();
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          swerve =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[0], driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[1], driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[2], driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      DriveConstants.MODULE_CONFIGS[3], driveSimulation.getModules()[3]));
          vision =
              new Vision(
                  new VisionIOPhotonvisionSim(
                      "arducam-4", 1, driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonvisionSim(
                      "arducam-5", 2, driveSimulation::getSimulatedDriveTrainPose));

          SimulatedArena.getInstance().resetFieldForAuto();
          climbPivot = new ClimbPivot(new ClimbPivotIOSim());
          climbRollers = new ClimbRollers(new ClimbRollersIOSim());

          intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
          intakePivot = new IntakePivot(new IntakePivotIOSim());
          l1Pivot = new L1Pivot(new L1PivotIOSim());
          intakeSensors = new IntakeSensors(new IntakeSensorIOSim(), new IntakeSensorIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          arm = new Arm(new ArmIOSim());
          clawRollers = new ClawRollers(new ClawRollersIOSim());
          objectDetection = new ObjectDetection(new ObjectDetectionIOSim());
        }
      }
    }

    // Swerve
    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // Vision
    if (vision == null) {
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
    }

    // CAN Watchdog
    if (canWatchdog == null) {
      canWatchdog = new CANWatchdog(new CANWatchdogIO() {}, rgb);
    }

    if (climbPivot == null) {
      climbPivot = new ClimbPivot(new ClimbPivotIO() {});
    }

    if (climbRollers == null) {
      climbRollers = new ClimbRollers(new ClimbRollersIO() {});
    }
    if (climbSensors == null) {
      climbSensors = new ClimbSensors(new ClimbSensorIOBeambreak() {});
    }

    if (rgb == null) {
      rgb = new RGB(new RGBIO() {});
    }
    climbController = new ClimbController(climbRollers, climbPivot, climbSensors);

    if (clawRollers == null) {
      clawRollers = new ClawRollers(new ClawRollersIO() {});
    }
    clawRollersController = new ClawRollersController(clawRollers);

    if (intakeRollers == null) {
      intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
    }
    if (intakePivot == null) {
      intakePivot = new IntakePivot(new IntakePivotIO() {});
    }
    if (intakeSensors == null) {
      intakeSensors = new IntakeSensors(new IntakeSensorIO() {}, new IntakeSensorIO() {});
    }
    intakeController = new IntakeController(intakeRollers, intakePivot, intakeSensors);

    if (l1Pivot == null) {
      l1Pivot = new L1Pivot(new L1PivotIO() {});
    }
    l1PivotController = new L1PivotController(l1Pivot);

    // Superstructure
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (arm == null) {
      arm = new Arm(new ArmIO() {});
    }
    superstructureController = new SuperstructureController(elevator, arm);

    clawRollersController = new ClawRollersController(clawRollers);

    if (objectDetection == null) {
      objectDetection = new ObjectDetection(new ObjectDetectionIOLimelight());
    }

    nameCommands();
    configureAutos();
    configureBindings();
    configureMCP();
  }

  public void configureMCP() {
    MCPServer mcpServer = new MCPServer(this, 3000);
    mcpServer.start();

    MCPTools mcpTools = mcpServer.getTools();
    // register the custom tools

    // drive to point tool
    mcpTools.registerTool(
        "DriveToPoint",
        "Drives the robot to a specified field point",
        MCPToolSchemas.getDriveToPointScheama(),
        (JSONObject args) -> {
          double x = ((Number) args.get("x")).doubleValue();
          double y = ((Number) args.get("y")).doubleValue();
          double theta = ((Number) args.get("theta")).doubleValue();

          Pose2d targetPose = new Pose2d(x, y, new Rotation2d(theta));

          Command driveCommand =
              new FunctionalCommand(
                  () -> swerve.setTargetPosition(targetPose),
                  () -> {},
                  (interrupted) -> swerve.clearTargetPositionController(),
                  () -> {
                    // Check if we're close enough to the target
                    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
                    double distanceError =
                        currentPose.getTranslation().getDistance(targetPose.getTranslation());
                    double angleError =
                        Math.abs(
                            currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
                    return distanceError < 0.05
                        && angleError < Math.toRadians(5); // 5cm and 5 degrees tolerance
                  },
                  swerve);

          driveCommand.schedule();

          JSONObject result = MCPTools.successResponse();
          result.put(
              "message",
              "Driving to position: x="
                  + x
                  + ", y="
                  + y
                  + ", theta="
                  + Math.toDegrees(theta)
                  + "°");
          return result;
        });

    // get robot status tool
    mcpTools.registerTool(
        "GetRobotStatus",
        "Gets the current position of the robot and state of the superstructure",
        MCPTools.createInputSchema(new JSONObject()),
        (JSONObject args) -> {
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
          robotPose.put("rotation_radians", currentPose.getRotation().getRadians());
          result.put("robot_pose", robotPose);

          // Superstructure current state
          JSONObject superstructureInfo = new JSONObject();
          superstructureInfo.put("state", superstructureState.toString());
          superstructureInfo.put("at_target", atTarget);

          JSONObject currentPoseInfo = new JSONObject();
          currentPoseInfo.put(
              "elevator_height_inches", currentSuperstructurePose.elevatorHeight.in(Units.Inches));
          currentPoseInfo.put(
              "arm_angle_degrees", currentSuperstructurePose.armAngle.in(Units.Degrees));
          currentPoseInfo.put("arm_direction", currentSuperstructurePose.armDirection.toString());
          superstructureInfo.put("current_pose", currentPoseInfo);

          JSONObject targetPoseInfo = new JSONObject();
          targetPoseInfo.put(
              "elevator_height_inches", targetSuperstructurePose.elevatorHeight.in(Units.Inches));
          targetPoseInfo.put(
              "arm_angle_degrees", targetSuperstructurePose.armAngle.in(Units.Degrees));
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
          // Field is symmetric, so red reef is at (17.573 - 4.5, 4.025)
          Translation2d redReefCenter =
              new Translation2d(
                  RobotState.fieldSizeX - blueReefCenter.getX(), blueReefCenter.getY());
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
        });

    // set superstructure state tool
    mcpTools.registerTool(
        "SetSuperstructureState",
        "Sets the target state of the superstructure (elevator and arm)",
        MCPToolSchemas.getSetSuperstructureStateSchema(),
        (JSONObject args) -> {
          String stateString = (String) args.get("state");

          // Check if still zeroing
          if (superstructureController.getSuperstructureState()
              == SuperstructureController.SuperstructureState.ZEROING) {
            JSONObject error = new JSONObject();
            error.put("success", false);
            error.put(
                "error",
                "Superstructure is still zeroing. Please wait for zeroing to complete before setting state.");
            error.put("current_state", "ZEROING");
            return error;
          }

          // Try to parse the state
          SuperstructureController.SuperstructureState targetState;
          try {
            targetState =
                SuperstructureController.SuperstructureState.valueOf(stateString.toUpperCase());
          } catch (IllegalArgumentException e) {
            JSONObject error = new JSONObject();
            error.put("success", false);
            error.put("error", "Invalid state: " + stateString);
            error.put(
                "valid_states", "STOW, TOP, L2_ALGAE, L3_ALGAE, GROUND_ALGAE, BARGE_RIGHT, CLIMB");
            return error;
          }

          // Prevent setting to ZEROING state
          if (targetState == SuperstructureController.SuperstructureState.ZEROING) {
            JSONObject error = new JSONObject();
            error.put("success", false);
            error.put("error", "Cannot manually set ZEROING state");
            return error;
          }

          // Schedule the command to set the state and wait for completion
          Command setSuperstructureCommand =
              superstructureController.setSuperstructureStateCommand(targetState);
          setSuperstructureCommand.schedule();

          JSONObject result = MCPTools.successResponse();
          result.put("message", "Setting superstructure state to: " + targetState.toString());
          result.put("state", targetState.toString());

          return result;
        });

    // Register StopRobot tool (stops both drivebase and superstructure)
    mcpTools.registerTool(
        "StopRobot",
        "Stops all robot movement immediately. Stops both the drivebase and superstructure (elevator and arm).",
        MCPTools.createInputSchema(new JSONObject()), // No parameters needed
        (args) -> {
          // Stop the drivebase by setting speeds to zero
          swerve.setTrajectorySpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0));
          swerve.clearTargetPositionController();
          swerve.clearHeadingControl();

          JSONObject result = MCPTools.successResponse();
          result.put("message", "Robot stopped");
          return result;
        });

    // Register IntakeClosestObject tool (drives to and picks up detected coral)
    mcpTools.registerTool(
        "IntakeClosestObject",
        "Drives the robot to the closest detected coral object and intakes it. Uses object detection camera to locate the coral, then approaches and picks it up automatically.",
        MCPTools.createInputSchema(new JSONObject()), // No parameters needed
        (args) -> {
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
          intakeObjectCommand.schedule();

          JSONObject result = MCPTools.successResponse();
          result.put("message", "Approaching and intaking coral");
          result.put("target_x", targetPose.getX());
          result.put("target_y", targetPose.getY());
          result.put("distance_meters", distance);
          return result;
        });
  }

  public void containerMatchStarting() {
    // runs when match starts
    canWatchdog.matchStarting();
  }

  /** Use this method to define the named commands for all of the autos */
  private void nameCommands() {
    // Register Command Names
    NamedCommands.registerCommand(
        "Score_L1",
        new SequentialCommandGroup(
                new FunctionalCommand(
                    () -> intakeController.setTargetStateCommand(IntakeState.L1),
                    () -> {},
                    (e) -> {},
                    () -> intakeController.intakeReachedTarget(),
                    intakeController))
            .withTimeout(2.6));
    NamedCommands.registerCommand(
        "Eject",
        new InstantCommand(() -> clawRollers.setVoltageTarget(ClawRollersTarget.EJECT_TOP)));
    NamedCommands.registerCommand(
        "Eject_L1",
        new SequentialCommandGroup(intakeController.setTargetStateCommand(IntakeState.EJECT)));
    NamedCommands.registerCommand("Zero", swerve.zeroGyroCommand());
    // NamedCommands.registerCommand(
    //     "L2_Algae",
    //     new InstantCommand(() ->
    // superstructureController.setSuperstructureStateCommand(SuperstructureState.L2_ALGAE)));
  }

  private void configureBindings() {
    configureAlgaeButtons();
    configureOverrideButtons();
    configureMultiUseButtons();
    configureL1Buttons();
    configureClimbButtons();

    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis(),
                      DriveConstants.DRIVE_CONFIG.maxLinearAcceleration());
                  if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                    swerve.clearHeadingControl();
                  }
                })
            .withName("Drive Teleop"));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    driverA.a().onTrue(new InstantCommand(() -> swerve.smartZeroGyro()));

    // align to the object
    driverB
        .leftTrigger()
        .whileTrue(
            new RunCommand(
                () -> {
                  if (objectDetection.coralInVision()) {
                    swerve.setTargetHeading(objectDetection.getTargetRotation());
                  }
                },
                objectDetection,
                swerve));

    // approach the object and intake
    driverB
        .rightTrigger()
        .whileTrue(new IntakeObjectCommand(swerve, objectDetection, intakeController));

    // auto align
    driverA
        .leftBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, true, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl())))
                .andThen(intakeController.setTargetStateCommand(IntakeController.IntakeState.L1))
                .andThen(new WaitCommand(1))
                .andThen(
                    intakeController.setTargetStateCommand(
                        IntakeController.IntakeState.UPRIGHT_INTAKE)));
    // auto align
    driverA
        .rightBumper()
        .whileTrue(
            (new ApproachReef(() -> levelOffsets, false, swerve)
                    .alongWith(new InstantCommand(() -> swerve.clearHeadingControl())))
                .andThen(intakeController.setTargetStateCommand(IntakeController.IntakeState.L1))
                .andThen(new WaitCommand(1))
                .andThen(
                    intakeController.setTargetStateCommand(
                        IntakeController.IntakeState.UPRIGHT_INTAKE)));
    new Trigger(
            () -> (Math.abs(driverA.getRightY()) > 0.2) || (Math.abs(driverA.getRightX()) > 0.2))
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.setTargetHeading(
                        calculateSnapTargetHeading(
                            new Rotation2d(
                                Math.atan2(
                                    MathUtil.applyDeadband(-driverA.getRightX(), 0.1),
                                    MathUtil.applyDeadband(-driverA.getRightY(), 0.1))))),
                interrupted -> {},
                () -> false));
  }

  private void configureClimbButtons() {
    driverB
        .b()
        .onTrue(
            climbController
                .setTargetCommand(ClimbController.ClimbState.INTAKE)
                .alongWith(
                    superstructureController.setTargetSuperstructureState(
                        SuperstructureController.SuperstructureState.CLIMB))
                .alongWith(
                    intakeController.setTargetStateCommand(IntakeController.IntakeState.CLIMB))
                .alongWith(
                    l1PivotController.setTargetStateCommand(L1PivotController.L1PivotState.CLIMB)));
    driverB.y().onTrue(climbController.setTargetCommand(ClimbController.ClimbState.CLIMB));
    driverB
        .x()
        .onTrue(
            climbController
                .setTargetCommand(ClimbController.ClimbState.IDLE)
                .alongWith(
                    new InstantCommand(
                        () ->
                            clawRollersController.setClawTarget(
                                ClawRollersController.ClawState.IDLE))));
  }

  private void configureL1Buttons() {
    driverA.x().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.INTAKE));
    driverA.y().onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.IDLE));
    driverB
        .leftTrigger()
        .onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.FORCE_INTAKE));
    driverB
        .leftTrigger()
        .onFalse(intakeController.setTargetStateCommand(IntakeController.IntakeState.INTAKE));
    driverB.leftBumper().onTrue(new ScoreL1Command(intakeController, l1PivotController));
  }

  private void configureMultiUseButtons() {
    // outtake

  }

  private void configureOverrideButtons() {
    driverB
        .a()
        .onTrue(
            superstructureController.setSuperstructureStateCommand(SuperstructureState.ZEROING));
    driverB
        .rightBumper()
        .onTrue(intakeController.setTargetStateCommand(IntakeController.IntakeState.HOLD));
    // driverB
    //     .leftBumper()
    //     .onTrue(superstructureController.setSuperstructureStateCommand(SuperstructureState.TOP));
  }

  private void configureAlgaeButtons() {
    driverB
        .povDown()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.GROUND_ALGAE)));
    driverB
        .povRight()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.L2_ALGAE)));
    driverB
        .povLeft()
        .onTrue(
            new InstantCommand(() -> clawRollersController.setClawTarget(ClawState.INTAKE))
                .alongWith(
                    superstructureController.setSuperstructureStateCommand(
                        SuperstructureState.L3_ALGAE)));

    driverB
        .povUp()
        .onTrue(
            superstructureController.setSuperstructureStateCommand(
                SuperstructureState.BARGE_RIGHT));

    driverB
        .rightBumper()
        .onTrue(superstructureController.setSuperstructureStateCommand(SuperstructureState.STOW));
    driverA
        .a()
        .onTrue(
            new InstantCommand(
                    () ->
                        clawRollersController.setClawTarget(
                            ClawRollersController.ClawState.EJECT_TOP))
                .andThen(new WaitCommand(2))
                .andThen(
                    new InstantCommand(
                        () ->
                            clawRollersController.setClawTarget(
                                ClawRollersController.ClawState.IDLE))));
  }

  private void configureAutos() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround

    BooleanSupplier flipAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance ll flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        () -> swerve.getRobotSpeeds(),
        (speeds) -> {
          swerve.setTrajectorySpeeds(speeds);
        },
        DriveConstants.HOLONOMIC_DRIVE_CONTROLLER,
        passRobotConfig,
        flipAlliance,
        swerve);

    autoChooser =
        new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
  }

  public Command getAutoCommand() {
    return autoChooser.get();
  }
  // runs when auto starts
  public void autoInit() {
    // Smart zero the robot
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> swerve.smartZeroGyro()));
  }

  // runs when teleop starts
  public void teleopInit() {
    CommandScheduler.getInstance()
        .schedule(new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 5, .5)));

    // vibrate controller at 30 seconds left
    CommandScheduler.getInstance()
        .schedule(
            new WaitCommand(105)
                .andThen(
                    new ParallelCommandGroup(new VibrateHIDCommand(driverB.getHID(), 3, 0.4))));
  }

  public void updateDashboardStatus() {
    // TODO: Define all of the dashboard outputs here
    SmartDashboard.putString("Current Auto", autoChooser.get().getName());
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = ((currentAngle - newAngle) % 360 + 360) % 360;
    double b = ((currentAngle - newAngle) % 360 + 360) % 360;
    return a < b ? a : -b;
  }

  public static Rotation2d calculateSnapTargetHeading(Rotation2d targetHeading) {

    targetHeading = targetHeading.rotateBy(Rotation2d.kPi); // because back of robot

    // TODO: Maybe make this *** mathematica ***
    double closest = DriveConstants.REEF_SNAP_ANGLES[0];
    for (double snap : DriveConstants.REEF_SNAP_ANGLES) {

      if (Math.abs(relativeAngularDifference(targetHeading.getDegrees(), snap))
          < Math.abs(relativeAngularDifference(targetHeading.getDegrees(), closest))) {

        closest = snap;
      }
    }

    return new Rotation2d(Math.toRadians(closest));
  }

  public void updateSimulation() {

    if (Constants.getRobotMode() != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput(
        "FieldSimulation/RobotPosition",
        RobotSimState.getInstance().getDriveSimulation().getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
