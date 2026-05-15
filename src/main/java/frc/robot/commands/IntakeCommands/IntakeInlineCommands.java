// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;

/** Add your docs here. */
public class IntakeInlineCommands {
  public Command fastRollers() {
      return Commands.runOnce(
          () -> RobotContainer.intake.setDesiredRollerState(RollerState.FAST),
          RobotContainer.intake);
    }   

  public  Command slowRollers () {
      return Commands.runOnce(
          () -> RobotContainer.intake.setDesiredRollerState(RollerState.SLOW),
          RobotContainer.intake);
    }

  public Command reverseRollers() {
      return Commands.runOnce(
          () -> RobotContainer.intake.setDesiredRollerState(RollerState.REVERSE),
          RobotContainer.intake);
    }

  public  Command stopRollers() {
      return Commands.runOnce(
          () -> RobotContainer.intake.setDesiredRollerState(RollerState.STOP),
          RobotContainer.intake);
    }

  public Command stowPivot() {
      return Commands.runOnce(
          () -> RobotContainer.intake.setDesiredPivotState(PivotState.STOW), RobotContainer.intake);
    }

  public Command deployedPivot() {
      return Commands.runOnce(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING),
              RobotContainer.intake)
          .until(
              () ->
                  RobotContainer.intake.getPivotPosition() >= PivotState.DEPLOYING.position - 0.05)
          .andThen(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYED),
              RobotContainer.intake);
    }

  private Debouncer pivotDebouncer = new Debouncer(0.5);
  public Command zeroIntake() {
      return Commands.run(
          () -> {
            RobotContainer.intake.setPivotVoltage(-1.5);
            RobotContainer.intake.setDesiredPivotState(PivotState.OFF);
          },
          RobotContainer.intake)
          .until(
            () -> pivotDebouncer.calculate(Math.abs(RobotContainer.intake.getPivotVelocity()) < 0.05))
          .andThen(
            () -> {
              RobotContainer.intake.setPivotEncoderAngle(0.0);
              RobotContainer.intake.setPivotVoltage(0.0);
              RobotContainer.intake.establishPivotZero(true);
              RobotContainer.intake.setDesiredPivotState(PivotState.STOW);
            }, RobotContainer.intake);
  }

  public static Command joltPivot () {
      return Commands.run(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.UP),
              RobotContainer.intake)
          .andThen(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING),
              RobotContainer.intake)
          .until(
              () ->
                  RobotContainer.intake.getPivotPosition() >= PivotState.DEPLOYING.position - 0.05)
          .andThen(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYED),
              RobotContainer.intake);
  }
}
