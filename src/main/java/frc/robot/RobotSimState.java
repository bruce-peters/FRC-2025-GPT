package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.RobotType;
import frc.robot.subsystems.swerve.DriveConstants;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotSimState {

  // SIMULATION VARIABLES
  private SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(
          DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  // Intake sim variables
  @AutoLogOutput(key = "RobotSimState/Intake Has Coral")
  private boolean intakeHasCoral;

  public void coralIntaked() {
    intakeHasCoral = true;
  }

  public boolean getIntakeHasCoral() {
    return intakeHasCoral;
  }

  private static RobotSimState instance;

  public static RobotSimState getInstance() {
    if (Constants.getRobotType() != RobotType.SIM) {
      System.err.println("ROBOT SHOULD NOT ACCESS SIM STATE IN NORMAL MODE");
    }
    if (instance == null) {
      instance = new RobotSimState();
    }
    return instance;
  }

  public RobotSimState() {
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
  }
}
