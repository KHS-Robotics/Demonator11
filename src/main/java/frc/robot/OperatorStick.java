// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class OperatorStick extends Joystick {
  public OperatorStick(int port) {
    super(port);
  }

  public double getElevatorSpeed() {
    return this.getRawAxis(1);
  }

  public double getPivotSpeed() {
    return this.getRawAxis(0);
  }
  
  public boolean decrementRPM() {
    return this.getRawButton(13);
  }

  public boolean incrementRPM() {
    return this.getRawButton(15);
  }

  public boolean resetRPM() {
    return this.getRawButton(11);
  }

  public boolean enableLimelight() {
    return this.getRawButton(0);
  }

  public boolean disableLimelight() {
    return this.getRawButton(2);
  }

  public boolean allowClimb() {
    return this.getRawAxis(3) > 0.1;
  }
}
