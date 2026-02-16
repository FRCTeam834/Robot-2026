package frc.robot.subsystems.indexer;

import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean indexerConnected = false;
    public double indexerRPM = 0.0;
    public double indexerAppliedVoltage = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerRPM(double targetRPM) {}

  public default void setIndexerVoltage(double targetVolts) {}

  public default void updateIndexerPID(SparkFlexConfig indexerConfig) {}

  public default void updateIndexerFeedforward(double kS, double kV) {}
}
