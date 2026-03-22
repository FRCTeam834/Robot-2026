// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Flywheel;


public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(1);
  private final Flywheel flywheel = new Flywheel();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    flywheel.setDefaultCommand(flywheel.joystickDriveCommand(joystick::getLeftY));

    joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
      * Joystick Y = quasistatic forward
      * Joystick A = quasistatic reverse
      * Joystick B = dynamic forward
      * Joystick X = dyanmic reverse
      */
    joystick.y().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    joystick.a().whileTrue(flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    joystick.b().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    joystick.x().whileTrue(flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }

  public Command getAutonomousCommand() {
      return Commands.runOnce(() -> {});
  }
}
