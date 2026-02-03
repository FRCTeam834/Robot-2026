// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import com.ctre.phoenix6.configs.Slot0Configs;

public class HoodIONeo extends SubsystemBase {

  // Motor Models
  private static final DCMotor hoodMotorModel = DCMotor.getNeo550(1); // Possibly Change Later

   //Create Sim; CHANGE PLACEHOLDER VALUES LATER
   private static final SingleJointedArmSim hoodSim = 
        new SingleJointedArmSim(
          hoodMotorModel,
          1.0, .004, .33, 0.0,
          Units.degreesToRadians(45),
          false,
          0,
          null); // Placeholder Values for now

  // Variables
  private double hoodVolts = 0.0; 

  // Constructor
  public HoodIONeo() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    //Update sim state
    hoodSim.update(0.02); //if we add a constants folder, lable this as double loopPeriodSecs = 0.05 inside the constants file

    // Hood inputs
    inputs.hoodConnected = true;
    inputs.hoodPositionRads = hoodSim.getAngleRads();
    inputs.hoodVelcoityRadsPerSec = hoodSim.getVelocityRadPerSec();
    inputs.hoodAppliedVoltage = hoodVolts;
  }

  @Override
  public void setHoodVoltage(double volts){
    this.hoodVolts = MathUtil.clamp(volts, -12.0, 12.0);
    hoodSim.setInputVoltage(this.hoodVolts);
  }

  @Override
  public void updateHoodPID(Slot0Configs config) {
    driveTalon.getConfigurator().apply(config); //FIX PID IMPLEMENTATION
  }
}


