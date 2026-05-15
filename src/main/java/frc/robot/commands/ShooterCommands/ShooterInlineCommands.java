package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.kicker.Kicker;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class ShooterInlineCommands {

  public static Command idleShooter(Flywheel flywheel) {
    return Commands.run(() -> flywheel.setDesiredFlywheelState(FlywheelState.IDLE), flywheel);
  }

  public static Command manualFlywheel(DoubleSupplier controllerJoystickY, Flywheel flywheel) {
    return Commands.run(
            () -> {
              flywheel.setManualVelocitySetpoint(
                  flywheel.getFlywheelVelocitySetpoint() + controllerJoystickY.getAsDouble() * 20);
            },
            flywheel)
        .beforeStarting(
            () -> {
              flywheel.setDesiredFlywheelState(FlywheelState.MANUAL_RPM);
              flywheel.setManualVelocitySetpoint(0.0);
            });
  }

  public static Command shootWhenReady(Flywheel flywheel, Kicker kicker, Intake intake, Drive drive, Vision vision, Supplier<Rotation2d> targetAngleSupplier, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return Commands.sequence(
      drive.alignToAngle(drive, targetAngleSupplier, xSupplier, ySupplier),

    );
  }
}
