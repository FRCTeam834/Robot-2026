package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.units.VoltageUnit;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.configs.Slot0Configs;

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

  public default void updateHoodPID(Slot0Configs config) {} // FIX PID IMPLEMENTATION
}
