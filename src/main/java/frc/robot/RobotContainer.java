// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.mcp.MCPTools;
import frc.robot.mcp.tools.DriveToPointTool;
import frc.robot.mcp.tools.GetRobotStatusTool;
import frc.robot.mcp.tools.IntakeClosestObjectTool;
import frc.robot.mcp.tools.MoveRelativeTool;
import frc.robot.mcp.tools.RotateRobotTool;
import frc.robot.mcp.tools.ScanForCoralTool;
import frc.robot.mcp.tools.SetSuperstructureStateTool;
import frc.robot.mcp.tools.StopRobotTool;
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

          intakePivot = new IntakePivot(new IntakePivotIOSim());
          intakeRollers = new IntakeRollers(new IntakeRollersIOSim(intakePivot));
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

  /**
   * Helper method to schedule a command and wait for it to finish. This is used by MCP tools to
   * ensure commands complete before returning.
   */
  public void scheduleAndWait(Command command) {
    command.schedule();

    // Wait for the command to finish (poll until it's no longer scheduled)
    // Use a spin-wait without sleep to avoid blocking the scheduler
    while (command.isScheduled()) {
      // Busy-wait - this is necessary because we're in a different thread
      // from the command scheduler. The scheduler runs in the main robot thread.
      Thread.yield(); // Give other threads a chance to run
    }
  }

  public void configureMCP() {
    MCPServer mcpServer = new MCPServer(this, 3000);
    mcpServer.start();

    MCPTools mcpTools = mcpServer.getTools();

    // Register all MCP tools
    mcpTools.registerTool(new DriveToPointTool(this, swerve));
    mcpTools.registerTool(new RotateRobotTool(this, swerve));
    mcpTools.registerTool(new MoveRelativeTool(this, swerve));
    mcpTools.registerTool(new GetRobotStatusTool(superstructureController, objectDetection));
    mcpTools.registerTool(new SetSuperstructureStateTool(this, superstructureController));
    mcpTools.registerTool(new StopRobotTool(swerve));
    mcpTools.registerTool(
        new IntakeClosestObjectTool(this, swerve, objectDetection, intakeController));
    mcpTools.registerTool(new ScanForCoralTool(this, swerve, objectDetection));
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
