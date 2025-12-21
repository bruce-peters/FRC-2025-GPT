package frc.robot.subsystems.claw;

import static frc.robot.subsystems.claw.ClawRollersConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ClawRollersIOSim extends GenericRollersIOSim implements ClawRollersIO {

  private final FlywheelSim ClawRollersSim;

  public ClawRollersIOSim() {
    super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    ClawRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), MOI, REDUCTION),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    ClawRollersSim.setInputVoltage(appliedVoltage);
    ClawRollersSim.update(0.02);

    double rotations = 0; // can't really be simulated
    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS = ClawRollersSim.getAngularVelocityRadPerSec() / (REDUCTION);

    // talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.connected = true;
    inputs.positionRads = rotations;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = talon.getSimState().getSupplyCurrent();

    // // Update robot sim state
    // RobotSimState.getInstance()
    //     .updateCoralPosition(MetersPerSecond.of(ROLLER_CIRCUMFERENCE * velocityRPS));
  }
}
