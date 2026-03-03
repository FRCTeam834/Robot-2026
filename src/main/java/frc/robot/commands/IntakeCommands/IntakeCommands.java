// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;

/** Add your docs here. */
public class IntakeCommands {
  public static Command fastRollers =
      Commands.run(
          () -> RobotContainer.intake.setRollerState(RollerState.FAST), RobotContainer.intake);

  public static Command slowRollers =
      Commands.run(
          () -> RobotContainer.intake.setRollerState(RollerState.SLOW), RobotContainer.intake);

  public static Command reverseRollers =
      Commands.run(
          () -> RobotContainer.intake.setRollerState(RollerState.REVERSE), RobotContainer.intake);

  public static Command stopRollers =
      Commands.run(
          () -> RobotContainer.intake.setRollerState(RollerState.STOP), RobotContainer.intake);

  public static Command deployIntake =
      Commands.run(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING),
              RobotContainer.intake)
          .alongWith(fastRollers).onlyIf(RobotContainer.intake::isPivotZeroed);

  public static Command retractIntake =
      Commands.run(
              () -> RobotContainer.intake.setDesiredPivotState(PivotState.UP),
              RobotContainer.intake)
          .alongWith(stopRollers).onlyIf(RobotContainer.intake::isPivotZeroed);;
}
