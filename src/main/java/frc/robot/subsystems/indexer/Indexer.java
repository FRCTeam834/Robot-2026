// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private IndexerState indexerState;

  public Indexer(IndexerIO io) {
    this.io = io;
    indexerState = IndexerState.STOP;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    setIndexerVoltage(indexerState.voltage);
  }

  // Setter Methods
  public void setIndexerVoltage(double targetVolts) {
    io.setIndexerVoltage(targetVolts);
  }

  public void setDesiredIndexerState(IndexerState indexerState) {
    setIndexerVoltage(indexerState.voltage);
  }

  // Getter Methods
  public double getIndexerVoltage() {
    return inputs.indexerAppliedVoltage;
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  public double getCurrent() {
    return inputs.indexerCurrent;
  }

  // Miscellaneous Method
  public void stopMotor() {
    io.stopMotor();
  }
}
