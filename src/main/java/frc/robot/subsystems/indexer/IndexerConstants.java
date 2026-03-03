// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

/** Add your docs here. */
public class IndexerConstants {
  public static enum IndexerState {
    FAST(12.0),
    SLOW(7.0),
    REVERSE(-7.0),
    STOP(0.0);

    public final double voltage;

    private IndexerState(double voltage) {
      this.voltage = voltage;
    }
  };
}
