package frc.robot.subsystems.shooter.hood;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.VoltageUnit;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodConnected = false;
    public double hoodPositionRads = 0.0;
    public double hoodVelocityRadsPerSec = 0.0;
    public double hoodAppliedVoltage = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodPosition(double positionRads, VoltageUnit feedforwardVolts) {}

  public default void setHoodVoltage(double volts) {}

  public default void updateHoodPID(
      SparkMaxConfig hoodConfig, double kS, double kV) {} // FIX PID IMPLEMENTATION
}
