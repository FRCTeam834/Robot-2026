// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.shooter.kicker.KickerConstants.KickerState;
import java.util.function.DoubleSupplier;

public class ShooterCommands {
  public static Command idleShooter(Flywheel flywheel) {
    return Commands.run(() -> flywheel.setDesiredState(FlywheelState.IDLE), flywheel);
  }

  public static Command dumbFlywheel(DoubleSupplier controllerJoystickY, Flywheel flywheel) {
    return Commands.run(
            () -> {
              flywheel.setVelocitySetpoint(
                  flywheel.getVelocitySetpoint() - controllerJoystickY.getAsDouble() * 20);
            },
            flywheel)
        .beforeStarting(
            () -> {
              flywheel.setDesiredState(FlywheelState.MANUAL_RPM);
              flywheel.setVelocitySetpoint(0.0);
            });
  }

  public static Command rampToRPM(double rpm, Flywheel flywheel) {
    return Commands.runOnce(
        () -> {
          flywheel.setVelocitySetpoint(rpm);
          flywheel.setDesiredState(FlywheelState.MANUAL_RPM);
        },
        flywheel);
  }

  public static Command shootWhenReadyManualVelocity(
      double rpm, Flywheel flywheel, Kicker kicker, Intake intake, Indexer indexer) {
    Debouncer flywheelReady = new Debouncer(0.25);
    return Commands.sequence(
            // clear the shooter
            Commands.runOnce(
                () -> {
                  kicker.setDesiredState(KickerState.STOP);
                  flywheel.setVelocitySetpoint(rpm);
                  flywheel.setDesiredState(FlywheelState.MANUAL_RPM);
                  intake.setDesiredRollerState(RollerState.STOP);
                },
                flywheel),
            Commands.waitUntil(() -> flywheelReady.calculate(flywheel.isAtSetpointRPM())),
            Commands.run(
                    () -> {
                      kicker.setDesiredState(KickerState.FEED);
                      indexer.setDesiredIndexerState(IndexerState.SLOW);
                    },
                    kicker,
                    indexer)
                .alongWith(IntakeCommands.shootingSequenceJolt().repeatedly()))
        .finallyDo(
            (interrupted) -> {
              kicker.setDesiredState(KickerState.STOP);
              indexer.setDesiredIndexerState(IndexerState.STOP);
              flywheel.setDesiredState(FlywheelState.IDLE);
              intake.setDesiredRollerState(RollerState.FAST);
              intake.setDesiredPivotState(PivotState.DEPLOYING);
            });
  }

  public static Command AutonShootWhenReadyManualVelocity(
      double rpm, Flywheel flywheel, Kicker kicker, Indexer indexer) {
    Debouncer flywheelReady = new Debouncer(0.25);
    return Commands.sequence(
            // clear the shooter
            Commands.runOnce(
                () -> {
                  kicker.setDesiredState(KickerState.STOP);
                  flywheel.setVelocitySetpoint(rpm);
                  flywheel.setDesiredState(FlywheelState.MANUAL_RPM);
                },
                flywheel),
            Commands.waitUntil(() -> flywheelReady.calculate(flywheel.isAtSetpointRPM())),
            Commands.run(
                    () -> {
                      kicker.setDesiredState(KickerState.FEED);
                      indexer.setDesiredIndexerState(IndexerState.SLOW);
                    },
                    kicker,
                    indexer)
                .alongWith(IntakeCommands.shootingSequenceJolt().repeatedly()))
        .finallyDo(
            (interrupted) -> {
              kicker.setDesiredState(KickerState.STOP);
              indexer.setDesiredIndexerState(IndexerState.STOP);
              flywheel.setDesiredState(FlywheelState.IDLE);
            });
  }

  public static Command shootWhenReady(
      boolean withYaw,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Flywheel flywheel,
      Kicker kicker,
      Intake intake,
      Indexer indexer,
      Drive drive) {
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
            withYaw
                ? Commands.waitUntil(() -> flywheelReady.calculate(flywheel.isAtSetpointRPM()))
                    .alongWith(
                        DriveCommands.AlignToAngleWithTolerance(
                            drive, xSupplier, ySupplier, drive::getFieldRelativeHUBAngle, 5))
                : Commands.waitUntil(() -> flywheelReady.calculate(flywheel.isAtSetpointRPM())),

            // phase 3, shoot and bring pivot up after there aren't as many balls in the hopper
            Commands.run(
                    () -> {
                      flywheel.setDesiredState(FlywheelState.ACTIVE);
                      kicker.setDesiredState(KickerState.FEED);
                      indexer.setDesiredIndexerState(IndexerState.FAST);
                      intake.setDesiredRollerState(RollerState.STOP);
                    },
                    flywheel,
                    kicker,
                    intake,
                    indexer)
                .alongWith(
                    Commands.waitSeconds(4)
                        .andThen(
                            Commands.runOnce(
                                () -> intake.setDesiredPivotState(PivotState.UP), intake))))

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
