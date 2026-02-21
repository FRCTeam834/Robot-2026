package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class KickerIOSparkMax implements KickerIO {
  private SparkFlex kickerMotor;
  private SparkFlexConfig kickerConfig;
  private SparkAbsoluteEncoder kickerEncoder;
  private SimpleMotorFeedforward kickerFeedforward;
  private SparkClosedLoopController kickerController;
  private AbsoluteEncoderConfig kickerEncoderConfig;

  public KickerIOSparkMax() {
    kickerMotor = new SparkFlex(10, null);
    kickerEncoder = kickerMotor.getAbsoluteEncoder();
    kickerConfig = new SparkFlexConfig();
    kickerFeedforward = new SimpleMotorFeedforward(0, 0);
    kickerController = kickerMotor.getClosedLoopController();
    kickerEncoderConfig = new AbsoluteEncoderConfig();


     //Kicker Config
    kickerConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0, 0, 0);

    kickerConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12.0);
    
    kickerEncoderConfig
    .positionConversionFactor(2 * Math.PI)
    .velocityConversionFactor(2 * Math.PI / 60);

    kickerConfig.apply(kickerEncoderConfig);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = true;
    inputs.kickerRPM = kickerEncoder.getVelocity();
    inputs.kickerAppliedVoltage = kickerMotor.getBusVoltage();
  }

  @Override
  public void setKickerVoltage(double targetVolts) {
    kickerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setKickerVelocity(double targetRPM) {
    double targetRotationsPerSec = targetRPM / 60;
    double ffVolts = kickerFeedforward.calculate(targetRotationsPerSec);
    kickerController.setSetpoint(
        targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void updateKickerPID(SparkFlexConfig config) {
    this.kickerConfig = config;
    kickerMotor.configure(
        kickerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateKickerFeedforward(double kS, double kV) {
    this.kickerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  @Override
  public void stopMotor() {
    kickerMotor.stopMotor();
  }
}
