package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;

public class KickerIOSparkMax implements KickerIO {
  private SparkFlex kickerMotor;
  private SparkFlexConfig kickerConfig;
  private SparkAbsoluteEncoder kickerEncoder;
  private AbsoluteEncoderConfig kickerEncoderConfig;

  public KickerIOSparkMax() {
    kickerMotor = new SparkFlex(10, null);
    kickerEncoder = kickerMotor.getAbsoluteEncoder();
    kickerConfig = new SparkFlexConfig();
    kickerEncoderConfig = new AbsoluteEncoderConfig();

    // Kicker Config
    kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);

    kickerEncoderConfig
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60);

    kickerConfig.apply(kickerEncoderConfig);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerRPM = kickerEncoder.getVelocity();
    inputs.kickerCurrent = kickerMotor.getOutputCurrent();
    inputs.kickerAppliedVoltage = kickerMotor.getBusVoltage();
  }

  @Override
  public void setKickerVoltage(double targetVolts) {
    kickerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
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
  public void stopMotor() {
    kickerMotor.stopMotor();
  }
}
