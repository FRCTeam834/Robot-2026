// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeCommands {
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

  public static Command toggleFastRollers =
      Commands.runOnce(
          () -> {
            if (RobotContainer.intake.getRollersState() == RollerState.FAST) {
              RobotContainer.intake.setDesiredRollerState(RollerState.STOP);
            } else {
              RobotContainer.intake.setDesiredRollerState(RollerState.FAST);
            }
          },
          RobotContainer.intake);

  public static Command deployIntake =
      Commands.sequence(
          new ZeroIntake(RobotContainer.intake).onlyIf(RobotContainer.intake::DoesPivotNeedZero),
          Commands.runOnce(
              () -> {
                RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING);
                RobotContainer.intake.setDesiredRollerState(RollerState.FAST);
              },
              RobotContainer.intake));

  public static Command retractIntake =
      Commands.sequence(
          new ZeroIntake(RobotContainer.intake).onlyIf(RobotContainer.intake::DoesPivotNeedZero),
          Commands.runOnce(
              () -> {
                RobotContainer.intake.setDesiredPivotState(PivotState.STOW);
                RobotContainer.intake.setDesiredRollerState(RollerState.STOP);
              },
              RobotContainer.intake));

  public static Command getJoltIntake() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotContainer.intake.setDesiredPivotState(PivotState.STOW);
            },
            RobotContainer.intake),
        Commands.waitSeconds(0.3),
        Commands.runOnce(
            () -> {
              RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING);
            },
            RobotContainer.intake));
  }

  public static Command shootingSequenceJolt() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              RobotContainer.intake.setDesiredPivotState(PivotState.STOW);
            },
            RobotContainer.intake),
        Commands.waitSeconds(0.4), // 0.4
        Commands.runOnce(
            () -> {
              RobotContainer.intake.setDesiredPivotState(PivotState.DEPLOYING);
            },
            RobotContainer.intake),
        Commands.waitSeconds(0.25)); // 0.25
  }

  public static Command dumbArm(DoubleSupplier controllerJoystickY, Intake intake) {
    double setpointAngle[] = new double[1];

    return Commands.run(
            () -> {
              setpointAngle[0] += controllerJoystickY.getAsDouble() * Math.PI * 0.05;
              setpointAngle[0] = MathUtil.clamp(setpointAngle[0], -0.1, Math.PI);
              intake.setPivotAngle(setpointAngle[0]);
              SmartDashboard.putNumber("DumbArmSetpoint", setpointAngle[0]);
            },
            intake)
        .beforeStarting(() -> setpointAngle[0] = intake.getCurrentPivotAngle());
  }
}
