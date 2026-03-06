package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double indexerAppliedVoltage;
    public double indexerCurrent;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerVoltage(double targetVolts) {}

  public default void stopMotor() {}
}
