package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Rollers
    public boolean rollerConnected = false;
    public double rollerRPM = 0.0;
    public double rollerAppliedVoltage = 0.0;

    // Pivot
    public boolean pivotConnected = false;
    public double pivotPositionRads = 0.0;
    public double pivotPositionRadsOffset = 0.0; // might have to be in terms of pi
    public double pivotAppliedVoltage = 0.0;
    public double pivotRPM = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double targetVolts) {}

  public default void setPivotVoltage(double volts) {}

  public default void setPivotAngle(double angle) {}

  public default void updateClosedLoopConfig(ClosedLoopConfig config) {}

  public default void stopMotors() {}
}
