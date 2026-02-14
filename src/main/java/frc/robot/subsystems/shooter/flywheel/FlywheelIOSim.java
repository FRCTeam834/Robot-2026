// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor flywheelMotorModel = DCMotor.getKrakenX60(9);

  public double flywheelVolts = 0.0;

  private final PIDController velocityPID = new PIDController(0, 0, 0);

  public FlywheelIOSim() {}

  private static final FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(flywheelMotorModel, .025, 1), flywheelMotorModel);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.flywheelConnected = true;
    // inputs.flywheelVelocityRadsPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVoltage = flywheelSim.getInputVoltage();
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    this.flywheelVolts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelSim.setInputVoltage(this.flywheelVolts);
  }

  @Override
  public void setFlywheelVelocity(double targetRPM, double ffVolts) {
    double targetRadPerSec = targetRPM * 2.0 * Math.PI / 60;
    double currentRadPerSec = flywheelSim.getAngularVelocityRadPerSec();

    // PID on velocity (rad/s)
    double pidVolts = velocityPID.calculate(currentRadPerSec, targetRadPerSec);

    // Use the ffVolts
    double appliedVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
    flywheelSim.setInputVoltage(appliedVolts);
  }
}
