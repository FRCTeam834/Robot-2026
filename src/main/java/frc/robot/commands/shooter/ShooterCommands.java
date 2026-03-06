// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;

/** Add your docs here. */
public class ShooterCommands {
  public static Command idleShooter() {
    return Commands.runOnce(
        () -> RobotContainer.flywheel.setDesiredState(FlywheelState.IDLE), RobotContainer.flywheel);
  }

  public static Command shootWhenReadyNoYaw(
      Flywheel flywheel, Kicker kicker, Intake intake, Indexer indexer) {
    Debouncer flywheelReady = new Debouncer(0.5);
    return Commands.sequence(
            // phase 1
            Commands.runOnce(
                () -> {
                  flywheel.setDesiredState(FlywheelState.ACTIVE);
                  kicker.setDesiredState(KickerState.STOP);
                  indexer.setDesiredIndexerState(IndexerState.SLOW);
                  intake.setDesiredRollerState(RollerState.SLOW);
                },
                flywheel,
                kicker,
                indexer,
                intake),

            // phase 2
            Commands.waitUntil(() -> flywheelReady.calculate(flywheel.isAtSetpointRPM())),

            // phase 3, shoot and bring pivot up after there aren't as many balls in the hopper
            Commands.run(
                () -> {
                  flywheel.setDesiredState(FlywheelState.ACTIVE);
                  kicker.setDesiredState(KickerState.FEED);
                  indexer.setDesiredIndexerState(IndexerState.FAST);
                  intake.setDesiredRollerState(RollerState.STOP);
                },
                flywheel, kicker, intake, indexer).alongWith(
                  Commands.runOnce(() -> intake.setDesiredPivotState(PivotState.UP), intake)
                  .onlyIf(() -> intake.getPivotCurrent() < 10))
                )
                
            // when command ends
            .finallyDo(
                (interrupted) -> {
                  kicker.setDesiredState(KickerState.STOP);
                  indexer.setDesiredIndexerState(IndexerState.STOP);
                  flywheel.setDesiredState(FlywheelState.IDLE);
                  intake.setDesiredRollerState(RollerState.STOP);
                  intake.setDesiredPivotState(PivotState.DEPLOYING);
                });
  }
}
