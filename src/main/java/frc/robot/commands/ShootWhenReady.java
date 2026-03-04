// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootWhenReady extends Command {
  private Flywheel flywheel;
  private Kicker kicker;
  private Indexer indexer;
  private Intake intake;
  private Drive drive;

  private Debouncer flywheelDebounce = new Debouncer(0.5);
  private Debouncer yawDebounce = new Debouncer(0.5);

  private boolean isFlywheelReady;
  private boolean isYawReady;

  public ShootWhenReady(Flywheel flywheel, Kicker kicker, Intake intake, Indexer indexer, Drive drive) {
    this.flywheel = flywheel;
    this.kicker = kicker;
    this.intake = intake;
    this.drive = drive;
    this.indexer = indexer;
    addRequirements(flywheel, kicker, intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.setDesiredState(FlywheelState.ACTIVE);
    kicker.setDesiredState(KickerState.STOP);
    indexer.setDesiredIndexerState(IndexerState.SLOW);
    intake.setDesiredRollerState(RollerState.STOP);

    isFlywheelReady = false;
    isYawReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // robot angle error to hub
    double yawError = drive.getFieldRelativeHUBAngle().minus(drive.getPose().getRotation()).getDegrees();

    isFlywheelReady = !flywheelDebounce.calculate(flywheel.isAtSetpointRPM());
    isYawReady = !yawDebounce.calculate(Math.abs(yawError) < 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kicker.setDesiredState(KickerState.FEED);
    indexer.setDesiredIndexerState(IndexerState.FAST);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFlywheelReady && isYawReady;
  }
}
