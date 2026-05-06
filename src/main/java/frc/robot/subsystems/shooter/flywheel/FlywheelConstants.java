package frc.robot.subsystems.shooter.flywheel;

public class FlywheelConstants {

  public static double idleRPM = 500;
  public static double toleranceRPM = 50.0;

  public enum FlywheelState {
    ACTIVE,
    MANUAL_RPM,
    IDLE,
    STOP
  };


}
