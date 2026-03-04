// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;

/** Add your docs here. */
public class IndexerCommands {

  public static Command fastBelt =
      Commands.runOnce(
          () -> RobotContainer.indexer.setDesiredIndexerState(IndexerState.FAST), RobotContainer.indexer);

  public static Command slowBelt =
      Commands.runOnce(
          () -> RobotContainer.indexer.setDesiredIndexerState(IndexerState.SLOW), RobotContainer.indexer);

  public static Command reverseBelt =
      Commands.runOnce(
          () -> RobotContainer.indexer.setDesiredIndexerState(IndexerState.REVERSE),
          RobotContainer.indexer);

  public static Command stopBelt =
      Commands.runOnce(
          () -> RobotContainer.indexer.setDesiredIndexerState(IndexerState.STOP), RobotContainer.indexer);
}
