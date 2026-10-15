package frc.robot.subsystems.intake.intake_rollers;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.RobotSimState;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.subsystems.intake.intake_pivot.IntakePivot;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {

  private final FlywheelSim intakeRollersSim;
  private final IntakeSimulation intakeSimulation;
  private final IntakePivot intakePivot;

  public IntakeRollersIOSim(IntakePivot intakePivot) {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    this.intakePivot = intakePivot;

    intakeRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION),
            DCMotor.getKrakenX60Foc(1));

    // Create intake simulation - back-mounted, 0.7m wide intake
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            RobotSimState.getInstance().getDriveSimulation(),
            Units.Meters.of(0.7),
            Units.Meters.of(0.3),
            IntakeSimulation.IntakeSide.BACK,
            // 1 // Can hold 1 coral at a time
            25 // testing value
            );

    // Set custom intake condition - only intake when pivot is 15 degrees or below
    intakeSimulation.setCustomIntakeCondition(
        (GamePieceOnFieldSimulation gamePiece) -> {
          // Get the current intake pivot angle in degrees (above horizontal)
          double pivotAngleDegrees = intakePivot.getPosition();

          // Only intake if pivot is at 15 degrees or below
          // Since the angle is measured above horizontal, values <= 15 mean the intake is down
          return pivotAngleDegrees <= 15.0;
        });
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    intakeRollersSim.setInputVoltage(appliedVoltage);
    intakeRollersSim.update(0.02);

    double rotations = 0; // can't really be simulated

    // Divides our angular velocity by our reduction
    double velocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() / REDUCTION;
    // FIXME: Doesn't work when reduction is 1

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRads = rotations;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated

    // Update intake simulation based on roller speed
    // Start intake if rollers are spinning forward (positive velocity)
    if (velocityRPS > 0.1) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }

    // Check if we collected a coral
    if (intakeSimulation.getGamePiecesAmount() > 0) {
      RobotSimState.getInstance().coralIntaked();
    }
  }
}
