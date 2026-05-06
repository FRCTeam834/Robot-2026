// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double lowerFlywheelVelocityRPM = 0.0;
    public double lowerFlywheelCurrent = 0.0;
    public double lowerFlywheelDutyCycle = 0.0;

    public double upperFlywheelVelocityRPM = 0.0;
    public double upperFlywheelCurrent = 0.0; 
    public double upperFlywheelDutyCycle = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setFlywheelVoltage(double targetVolts) {}

  public default void setFlywheelVelocity(double targetRPM) {}

  public default void updateFlywheelPID(Slot0Configs configs) {}

  public default void stopMotor() {}
}
