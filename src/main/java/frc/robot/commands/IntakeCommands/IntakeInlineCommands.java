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
    public static Command fastRollers = 
        Commands.runOnce(
        () -> RobotContainer.intake.setRollerState(RollerState.FAST), 
        RobotContainer.intake);

    public static Command slowRollers = 
        Commands.runOnce(
        () -> RobotContainer.intake.setRollerState(RollerState.SLOW), 
        RobotContainer.intake);

    public static Command reverseRollers = 
        Commands.runOnce(
        () -> RobotContainer.intake.setRollerState(RollerState.REVERSE), 
        RobotContainer.intake);

    public static Command stopRollers = 
        Commands.runOnce(
        () -> RobotContainer.intake.setRollerState(RollerState.STOP), 
        RobotContainer.intake);
}
