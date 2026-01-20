// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  public static final XboxController xbox = new XboxController(2);

  public static final double flightJoystickDeadzone = 0;
  public static final double xboxJoystickDeadzone = 0;

  public static final double getLeftJoystickX() {
    double raw = leftJoystick.getX();
    if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
    // return calculateUnitQuad(raw);
    return raw;
  }

  /**
   * @return left joystick y input
   */
  public static final double getLeftJoystickY() {
    double raw = leftJoystick.getY();
    if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
    // return calculateUnitQuad(raw);
    return raw;
  }

  /**
   * @return right joystick x input
   */
  public static final double getRightJoystickX() {
    double raw = rightJoystick.getX();
    if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
    // return calculateUnitQuad(raw);
    return raw;
  }

  /**
   * @return right joystick y input
   */
  public static final double getRightJoystickY() {
    double raw = rightJoystick.getY();
    if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
    // return calculateUnitQuad(raw);
    return raw;
  }

  public static final double calculateUnitQuad(double value) {
    return Math.copySign(value * value, value);
  }
}
