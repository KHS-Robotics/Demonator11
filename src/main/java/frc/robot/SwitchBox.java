/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class SwitchBox extends Joystick {
  SwitchBox(int port) {
    super(port);
  }

  public boolean intakeDown() {
    return !this.getRawButton(1);
  }

  public boolean positionControl() {
    return this.getRawButton(3);
  }

  public boolean rotationControl() {
    return this.getRawButton(2);
  }

  public boolean rampShooter() {
    return !this.getRawButton(4);
  }

  public boolean intake() {
    return this.getRawButton(5);
  }

  public boolean outtake() {
    return this.getRawButton(6);
  }

  public boolean shoot() {
    return this.getRawButton(12);
  }

  public boolean guide() {
    return !this.getRawButton(7);
  }

  public boolean climb() {
    return this.getRawButton(8);
  }

  public boolean shooterOverride() {
    return !this.getRawButton(11);
  }

  public boolean controlPanelOverride() {
    return !this.getRawButton(10);
  }

  public boolean engagePTO() {
    return !this.getRawButton(9);
  }

  public double getHoodSpeed() {
    return -this.getRawAxis(0) * 0.2;
  }

  public double getIndexSpeed() {
    return this.getRawAxis(1);
  }

  public double getTelescopeSpeed() {
    return this.getRawAxis(2) * 0.5;
  }

  public double getControlPanel() {
    return this.getRawAxis(3);
  }
}
