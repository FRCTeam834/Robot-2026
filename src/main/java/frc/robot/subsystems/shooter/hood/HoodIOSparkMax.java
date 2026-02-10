// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig; // trying to see if this works
// import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class HoodIOSparkMax implements HoodIO {
  private SparkMax hoodMotor;
  private SparkMaxConfig hoodConfig;
  // private SparkClosedLoopController hoodPID;
  private SparkAbsoluteEncoder absEncoder;

  private double hoodVolts = hoodMotor.getBusVoltage();
  private double velocity = absEncoder.getVelocity();
  private double positionRads = absEncoder.getPosition();

  public HoodIOSparkMax() {
    hoodMotor = new SparkMax(10, null);
    // hoodPID = hoodMotor.getClosedLoopController();
    absEncoder = hoodMotor.getAbsoluteEncoder();
    hoodConfig = new SparkMaxConfig();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodConnected = true;
    inputs.hoodPositionRads = positionRads;
    inputs.hoodVelocityRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocity);
    inputs.hoodAppliedVoltage = hoodVolts;
  }

  public void setHoodVoltage(double volts) {
    this.hoodVolts = MathUtil.clamp(volts, -12.0, 12.0);
    hoodMotor.setVoltage(this.hoodVolts);
  }

  // Make method for hood position

  public void updateHoodPID(SparkMaxConfig hoodMaxConfig, double kS, double kV) {
    this.hoodConfig = hoodMaxConfig;
    hoodMotor.configure(
        hoodConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
    // hoodPID.configure(hoodConfig, com.revrobotics.ResetMode.kNoResetSafeParameters,
    // com.revrobotics.PersistMode.kNoPersistParameters)

    /*configureSpark("", () -> { return hoodMotor.setP(kP); });
    configureSpark("", () -> { return hoodMotor.setS(kS); });
    configureSpark("", () -> { return hoodMotor.set(kV); }); //doesn't the set(kV) set a percent output and not feedforward?? */
  }
}
