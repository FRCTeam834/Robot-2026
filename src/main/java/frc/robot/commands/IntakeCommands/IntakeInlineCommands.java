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
    public static Command fastRollers =
        Commands.runOnce(
          () -> RobotContainer.intake.setDesiredRollerState(RollerState.FAST),
          RobotContainer.intake);

    public static Command slowRollers =
        Commands.runOnce(
            () -> RobotContainer.intake.setDesiredRollerState(RollerState.SLOW),
            RobotContainer.intake);

    public static Command reverseRollers =
        Commands.runOnce(
            () -> RobotContainer.intake.setDesiredRollerState(RollerState.REVERSE),
            RobotContainer.intake);

    public static Command stopRollers =
        Commands.runOnce(
            () -> RobotContainer.intake.setDesiredRollerState(RollerState.STOP),
            RobotContainer.intake);

    public static Command stowPivot =
        Commands.runOnce(
            () -> RobotContainer.intake.setDesiredPivotState(PivotState.STOW), RobotContainer.intake);

    public static Command deployPivot =
        Commands.runOnce(
            () -> RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOY),
            RobotContainer.intake);

    private static Debouncer pivotDebouncer = new Debouncer(0.5);    
    public static Command zeroIntake = 
            Commands.runOnce(
            () -> RobotContainer.intake
            )

    
    


    }
