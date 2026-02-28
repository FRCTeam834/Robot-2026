// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinFlywheel extends Command {
  private final Flywheel flywheel;
  private final DoubleSupplier rpmSupplier;

  /**
   * Spins the flywheel at a specific RPM.
   *
   * @param flywheel The flywheel subsystem.
   * @param rpmSupplier A supplier for the target RPM. Allows for RPM changes on the fly
   */
  public SpinFlywheel(Flywheel flywheel, DoubleSupplier rpmSupplier) {
    this.flywheel = flywheel;
    this.rpmSupplier = rpmSupplier;
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setFlywheelRPM(rpmSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
