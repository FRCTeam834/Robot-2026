// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;

/** Add your docs here. */
public class FlywheelConstants {
  public static enum FlywheelState {
    STOPPED,
    IDLE,
    ACTIVE,
    MANUAL_RPM
  }

  public static Slot0Configs flywheelConfig = new Slot0Configs().withKP(0).withKS(0).withKV(0);

  public static double idleRPM = 500.0;
  public static double toleranceRPM = 10.0;
}
