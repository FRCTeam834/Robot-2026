// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroIntake extends Command {
  /** Creates a new ZeroIntake. */
  private final Intake intake;

  private Debouncer veloDebouncer = new Debouncer(0.2);
  private boolean isVelocityZero = false;

  public ZeroIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setDesiredPivotState(PivotState.OFF);
    intake.setPivotVoltage(-1.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isVelocityZero = veloDebouncer.calculate(Math.abs(intake.getPivotVelocity()) < 0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setEncoderAngle(0);
    intake.setPivotVoltage(0);
    intake.establishPivotZero(true);
    intake.setDesiredPivotState(PivotState.STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isVelocityZero && Math.abs(intake.getPivotCurrent()) > 35);
  }
}
