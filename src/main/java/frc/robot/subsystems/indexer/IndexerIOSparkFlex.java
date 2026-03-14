// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IndexerIOSparkFlex implements IndexerIO {
  private SparkFlex indexerMotor;

  public IndexerIOSparkFlex() {
    indexerMotor = new SparkFlex(50, MotorType.kBrushless);
    SparkFlexConfig indexerConfig = new SparkFlexConfig();

    indexerConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(20)
        .voltageCompensation(12);

    indexerMotor.configure(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerCurrent = indexerMotor.getOutputCurrent();
    inputs.indexerAppliedVoltage = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
  }

  @Override
  public void setIndexerVoltage(double volts) {
    // indexerMotor.setVoltage(volts);
  }

  @Override
  public void stopMotor() {
    indexerMotor.stopMotor();
  }
}
