// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

/** Add your docs here. */
public class KickerConstants {
  public static enum KickerState {
    FEED(11.0),
    REVERSE(-3.0),
    STOP(0.0);

    public final double voltage;

    private KickerState(double voltage) {
      this.voltage = voltage;
    }
  };
}
