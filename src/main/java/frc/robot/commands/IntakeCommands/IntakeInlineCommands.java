// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;    
import frc.robot.subsystems.intake.Intake.RollerState;

/** Add your docs here. */
public class IntakeInlineCommands {
    public static Command startRollers = 
        Commands.startEnd(
        () -> RobotContainer.intake.setRollerState(RollerState.FAST), 
        () -> RobotContainer.intake.setRollerState(RollerState.STOP),
        RobotContainer.intake);
}
