// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import edu.wpi.first.math.filter.Debouncer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroIntake extends Command {

  private final Intake intake;
  private Debouncer pivotDebouncer = new Debouncer(0.5); // i think this would be fine as 0.1
  private boolean pivotAtZero;

  public ZeroIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPivotVoltage(-0.5);
  }

  // Called every time the scheduler runs while thes command is scheduled.
  @Override
  public void execute() {
    pivotAtZero = pivotDebouncer.calculate(intake.getPivotVelocity() < 0.05); //i saw someone else using rawSignal (variable)? helpful or not needed?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setEncoderAngle(IntakeConstants.intakeZeroAngle);
    intake.setDesiredPivotState(PivotState.STOW);
    intake.setPivotVoltage(0);
    intake.establishPivotZero(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivotAtZero;
  }
}
