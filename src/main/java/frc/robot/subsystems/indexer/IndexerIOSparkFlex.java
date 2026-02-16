// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IndexerIOSparkFlex implements IndexerIO {
  private SparkFlex indexerMotor;
  private SparkAbsoluteEncoder indexerEncoder;
  private SparkFlexConfig indexerConfig;
  private SparkClosedLoopController indexerController;
  private SimpleMotorFeedforward indexerFeedforward;

  public IndexerIOSparkFlex() {
    indexerMotor = new SparkFlex(13, MotorType.kBrushless);
    indexerEncoder = indexerMotor.getAbsoluteEncoder();
    indexerConfig = new SparkFlexConfig();
    indexerController = indexerMotor.getClosedLoopController();
    indexerFeedforward = new SimpleMotorFeedforward(0, 0);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerConnected = true;
    inputs.indexerRPM = indexerEncoder.getVelocity();
    inputs.indexerAppliedVoltage = indexerMotor.getBusVoltage();
  }

  @Override
  public void setIndexerRPM(double targetRPM) {
    double targetRPS = targetRPM / 60;
    double ffVolts = indexerFeedforward.calculate(targetRPS);
    indexerController.setSetpoint(
        targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void setIndexerVoltage(double targetVolts) {
    indexerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void updateIndexerPID(SparkFlexConfig config) {
    this.indexerConfig = config;
    indexerMotor.configure(
        indexerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateIndexerFeedforward(double kS, double kV) {
    this.indexerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }
}
