package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {

  // Motor Models
  private static final DCMotor hoodMotorModel =
      DCMotor.getKrakenX60(1); // Name or Number of motors?
  private static final DCMotor flywheelMotorModel = DCMotor.getKrakenX60(1);
  
  // Create Sim; CHANGE PLACEHOLDER VALUES LATER
  private static final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          hoodMotorModel,
          1.0,
          .004,
          .33,
          0.0,
          Units.degreesToRadians(45),
          false,
          0,
          null); // Placeholder Values for now
  private static final DCMotorSim flywheelSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(flywheelMotorModel, .025, 1),
          flywheelMotorModel); // Placeholder Values for now

  // Variables
  private double flywheelVolts = 0.0; // used to be flywheelAppliedVolts
  private double hoodVolts = 0.0; // used to be flywheelAppliedVolts

  // Constructor
  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    // Update sim state
    flywheelSim.update(
        0.02); // if we add a constants folder, lable this as double loopPeriodSecs = 0.05 inside
    // the constants file
    hoodSim.update(0.02);

    // Flywheel inputs
    inputs.flywheelConnected = true;
    inputs.flywheelVelocityRadsPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVoltage = flywheelVolts;

    // Hood inputs
    inputs.hoodConnected = true;
    inputs.hoodPositionRads = hoodSim.getAngleRads();
    inputs.hoodVelocityRadsPerSec = hoodSim.getVelocityRadPerSec();
    inputs.hoodAppliedVoltage = hoodVolts;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    this.flywheelVolts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelSim.setInputVoltage(this.flywheelVolts);
  }

  @Override
  public void setHoodVoltage(double volts) {
    this.hoodVolts = MathUtil.clamp(volts, -12.0, 12.0);
    hoodSim.setInputVoltage(this.hoodVolts);
  }
}
