// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.units.VoltageUnit;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelVelocityRadsPerSec = 0.0;
    public double flywheelAppliedVoltage = 0.0;
    // public double flywheelTorqueCurrentAmps = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setFlywheelVelocity(
      double velocityRadsPerSec, VoltageUnit feedforwardVolts) {}

  public default void setFlywheelVoltage(double volts) {}
}
