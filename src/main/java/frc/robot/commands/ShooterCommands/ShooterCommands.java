// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;

/** Add your docs here. */
public class ShooterCommands {
  public static Command idleShooter() {
    return Commands.run(() -> RobotContainer.flywheel.setDesiredState(FlywheelState.IDLE), RobotContainer.flywheel);
  }
}
