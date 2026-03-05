// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private final Flywheel flywheel;
  private final Vision vision;
  private final Drive drive;
  private final LinearFilter shooterAngleAverage = LinearFilter.movingAverage(10);
  private final Timer timer = new Timer();

  public AutoAlign(Flywheel flywheel, Vision vision, Drive drive){
    this.flywheel = flywheel;
    this.vision = vision;
    this.drive = drive;
    addRequirements(flywheel, vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.stopMotor();
    timer.reset();
    timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(vision.getTargetX(0) != null) {
      timer.reset();
      timer.stop();
    } else {
      timer.start();
    }

  if (!timer.hasElapsed(1.0)) {
    double error = vision.get
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
