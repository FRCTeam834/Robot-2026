package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class KickerIOSparkMax implements KickerIO{
  private SparkFlex kickerMotor;
  private SparkFlexConfig kickerConfig;
  private SparkAbsoluteEncoder absEncoder;
  private SimpleMotorFeedforward kickerFeedforward;
  private SparkClosedLoopController kickerController;

  public KickerIOSparkMax() {
    kickerMotor = new SparkFlex(10, null);
    absEncoder = kickerMotor.getAbsoluteEncoder();
    kickerConfig = new SparkFlexConfig();
    kickerFeedforward = new SimpleMotorFeedforward(0,0);    
    kickerController = kickerMotor.getClosedLoopController();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = true;
    inputs.kickerRPM = absEncoder.getVelocity();
   }

  public void setKickerVelocity(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = kickerFeedforward.calculate(targetRotationsPerSec);
    kickerController.setSetpoint(targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  public void updateKickerPID(SparkFlexConfig config) {
    this.kickerConfig = config;
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
