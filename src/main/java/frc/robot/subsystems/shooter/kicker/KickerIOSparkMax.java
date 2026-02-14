package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;


public class KickerIOSparkMax implements KickerIO{
  private SparkFlex kickerMotor;
  private SparkFlexConfig kickerConfig;
  private SparkAbsoluteEncoder absEncoder;
  public SimpleMotorFeedforward kickerFeedforward;
  // private final VelocityVoltage velocitySetPoint; // CHANGE TO NEO Language; currently in phoenix

//   private double kickerVolts = kickerMotor.getBusVoltage();
  private double currentRotationsPerMinute = absEncoder.getVelocity();

  public KickerIOSparkMax() {
    kickerMotor = new SparkFlex(10, null);
    absEncoder = kickerMotor.getAbsoluteEncoder();
    kickerConfig = new SparkFlexConfig();
    kickerFeedforward = new SimpleMotorFeedforward(0,0);    
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = true;
    inputs.kickerRPM = currentRotationsPerMinute;
   }

  public void setKickerVelocity(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = kickerFeedforward.calculate(targetRotationsPerSec);

    }

  public void updateKickerPID(SparkFlexConfig kickerFlexConfig) {
    this.kickerConfig = kickerFlexConfig;
    kickerMotor.configure(
        kickerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateKickerFeedforward(double kS, double kV){
    this.kickerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

}
