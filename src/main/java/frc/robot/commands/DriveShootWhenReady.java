// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveShootWhenReady extends Command {
  /** Creates a new DriveShootWhenReady. */
  private final Flywheel flywheel;
  private final Kicker kicker;
  private final Intake intake;
  private final Drive drive;
  private final Indexer indexer;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private final PIDController alignController = new PIDController(1, 0, 0);

  public DriveShootWhenReady(
    Flywheel flywheel, 
    Kicker kicker, 
    Drive drive, 
    Intake intake, 
    Indexer indexer,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier) {
    this.flywheel = flywheel;
    this.kicker = kicker;
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;

    addRequirements(flywheel, kicker, drive, intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveCommands.joystickDrive(drive, xSupplier, ySupplier, () -> { return alignController.calculate(getTargetHubAngleError()); });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getTargetHubAngleError() {
    Pose2d robotPose = drive.getPose();
    Translation2d goalPose = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Translation2d robotLocation = robotPose.getTranslation();
    Translation2d targetVector = goalPose.minus(robotLocation);

    Rotation2d fieldRelativeRobotAngle = robotPose.getRotation();
    return targetVector.getAngle().minus(fieldRelativeRobotAngle).getDegrees();
  }
}
