// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodIOSparkMax implements HoodIO {
  private CANSparkMax 


  // Motor Models
  private static final DCMotor hoodMotorModel = DCMotor.getNeo550(1); // Possibly Change Later

  // Variables
  private double hoodVolts = 0.0;

  // Constructor
  public HoodIOSparkMax() {}

  @Override
  public void updateInputs(HoodIOSparkMax inputs) {
    // Update sim state
    hoodSim.update(0.02); // if we add a constants folder, lable this as double loopPeriodSecs = 0.05 inside
    // the constants file

    // Hood inputs
    inputs.hoodConnected = true;
    inputs.hoodPositionRads = hoodSim.getAngleRads();
    inputs.hoodVelcoityRadsPerSec = hoodSim.getVelocityRadPerSec();
    inputs.hoodAppliedVoltage = hoodVolts;
  }

  @Override
  public void setHoodVoltage(double volts) {
    this.hoodVolts = MathUtil.clamp(volts, -12.0, 12.0);
    hoodSim.setInputVoltage(this.hoodVolts);
  }

  @Override
  public void updateHoodPID(Slot0Configs config) {
    hoodMotorModel.getConfigurator().apply(config); // FIX PID IMPLEMENTATION
  }

}
