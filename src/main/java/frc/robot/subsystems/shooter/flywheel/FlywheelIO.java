// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean flywheelConnected = false;
    public double flywheelVelocityRPM;
    public double flywheelCurrent = 0.0;
    public double flywheelDutyCycle = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setFlywheelVelocity(double RPM) {}

  public default void setFlywheelDutyCycle(double dutyCycle) {}

  public default void updateClosedLoopConfig(Slot0Configs config) {}

  public default void stopMotor() {}
}
