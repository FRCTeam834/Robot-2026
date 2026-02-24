package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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

  public default void setRollerRPM(double targetRPM) {}

  public default void updateRollerPID(SparkFlexConfig rollerConfig) {}

  public default void updateRollerFeedforward(double kS, double kV) {}

  public default void setPivotVoltage(double volts) {}

  public default void setPivotPosition(double targetPosition) {}

  public default void updatePivotPID(SparkMaxConfig pivotConfig) {}

  public default void updatePivotFeedforward(double kS, double kG, double kV) {}

  public default void stopMotors() {}
}
