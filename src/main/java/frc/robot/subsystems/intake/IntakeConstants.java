package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static enum RollerState {
    FAST(8.0),
    SLOW(3.0),
    REVERSE(-3.0),
    STOP(0.0);

    public final double voltage;

    private RollerState(double voltage) {
      this.voltage = voltage;
    }
  };

  public static enum PivotState {
    STOW(0.0),
    DEPLOYING(1), // GOING TO CHANGE LATER
    DEPLOY(Math.toRadians(90)); // GOING TO CHANGE LATER

    public final double position;

    private PivotState(double position) {
      this.position = position;
    }
  };
}
