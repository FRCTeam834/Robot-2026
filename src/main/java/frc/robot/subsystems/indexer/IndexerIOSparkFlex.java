// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;

public class IndexerIOSparkFlex implements IndexerIO {
  private SparkFlex indexerMotor;
  public SparkFlexConfig indexerConfig;

  public IndexerIOSparkFlex() {
    indexerMotor = new SparkFlex(13, MotorType.kBrushless);
    indexerConfig = new SparkFlexConfig();

    indexerConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerConnected = true;
    inputs.indexerAppliedVoltage = indexerMotor.getAppliedOutput()*indexerMotor.getBusVoltage();
  }

  @Override
  public void setIndexerVoltage(double targetVolts) {
    indexerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void stopMotor() {
    indexerMotor.stopMotor();
  }

}
