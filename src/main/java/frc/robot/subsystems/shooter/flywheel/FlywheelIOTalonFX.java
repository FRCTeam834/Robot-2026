// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final DCMotor flywheelMotorModel = DCMotor.getKrakenX60(1);

  public FlywheelIOTalonFX() {}

  private static final FlywheelSim flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelMotorModel, .025, 1), flywheelMotorModel);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // need to fix implementation
    inputs.flywheelConnected = true;
    inputs.flywheelVelocityRadsPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.flywheelAppliedVoltage = flywheelSim.getInputVoltage();
  }
}
