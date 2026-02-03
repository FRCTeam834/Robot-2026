// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;
import edu.wpi.first.math.system.plant.LinearSystemId;


public class FlywheelIOTalonFX implements FlywheelIO {
  private static final DCMotor flywheelMotorModel = DCMotor.getKrakenX60(1); 

  public FlywheelIOTalonFX(String name) {}
  private static final FlywheelSim flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelMotorModel, .025, 1),flywheelMotorModel);  
  this.flywheelMotorModel = flywheelMotorModel;
  public void updateInputs(FlywheelIOInputs inputs) {
    //need to fix implementation
    this.flywheelConnected = true;
    this.flywheelVelocityRadsPerSec = flywheelSim.getAngularVelocityRadPerSec();
    this.flywheelAppliedVoltage = flywheelSim.getInputVoltage();
  }

}
