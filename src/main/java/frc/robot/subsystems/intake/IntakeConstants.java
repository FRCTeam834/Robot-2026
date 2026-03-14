// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeConstants {
  public static enum RollerState {
    FAST(8.0),
    SLOW(3.0),
    REVERSE(-3.0),
    STOP(0.0);

    public final double voltage;

    private RollerState(double voltage) {
      this.voltage = voltage;
    }
  };

  public static enum PivotState {
    STOW((0.05)),
    UP((0.3)),
    DEPLOYING((2.15)),
    DEPLOYED(2.15),
    OFF((0));

    public final double position;

    private PivotState(double position) {
      this.position = position;
    }
  }

  public static double pivotTolerance = Math.toRadians(2);
}
