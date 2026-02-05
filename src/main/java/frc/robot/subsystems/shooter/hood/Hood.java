// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private static final LoggedTunableNumber hood_kP = new LoggedTunableNumber("Hood/hood_kP");
  private static final LoggedTunableNumber hood_kS = new LoggedTunableNumber("Hood/hood_kS");
  private static final LoggedTunableNumber hood_kV = new LoggedTunableNumber("Hood/hood_kV");

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (Constants.tuningMode && hood_kP.hasChanged(hashCode()) && hood_kS.hasChanged(hashCode()) &&  hood_kV.hasChanged(hashCode())){
      var hoodConfig = new Slot0Configs();
      hoodConfig.kP = hood_kP.get();
      io.updateHoodPID(hoodConfig); // FIX PID IMPLEMENTATION
    }
  }
}
