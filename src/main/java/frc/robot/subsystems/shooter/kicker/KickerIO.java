package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.config.SparkFlexConfig;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double kickerRPM = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVoltage(double targetVolts) {}

  public default void stopMotor() {}
}
