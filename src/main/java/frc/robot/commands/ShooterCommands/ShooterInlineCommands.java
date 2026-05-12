package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import java.util.function.DoubleSupplier;

public class ShooterInlineCommands {

  public static Command idleShooter(Flywheel flywheel) {
    return Commands.run(() -> flywheel.setDesiredFlywheelState(FlywheelState.IDLE), flywheel);
  }

  public static Command manualFlywheel(DoubleSupplier controllerJoystickY, Flywheel flywheel) {
    return Commands.run(
            () -> {
              flywheel.setManualVelocitySetpoint(
                  flywheel.getFlywheelSetpointRPM() - controllerJoystickY.getAsDouble() * 20);
            },
            flywheel)
        .beforeStarting(
            () -> {
              flywheel.setDesiredFlywheelState(FlywheelState.MANUAL_RPM);
              flywheel.setManualVelocitySetpoint(0.0);
            });
  }
}
