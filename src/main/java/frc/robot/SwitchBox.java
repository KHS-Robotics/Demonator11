package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class SwitchBox extends Joystick {

  public SwitchBox(int port) {
    super(port);
  }

  public boolean intakeDown() {
    return this.getRawButton(3);
  }

  public boolean rampShooter() {
    return this.getRawButton(8);
  }

  public boolean eject() {
    return this.getRawButton(7);
  }

  public boolean stopAutoEject() {
    return this.getRawAxis(0) > 0.1;
  }

  public boolean intake() {
    return this.getRawAxis(1) > 0.75;
  }

  public boolean outtake() {
    return this.getRawAxis(1) < -0.75;
  }

  public boolean shoot() {
    return this.getRawButton(6);
  }

  public boolean climb() {
    return this.getRawButton(5);
  }

  public boolean climbOverride() {
    return this.getRawButton(12);
  }

  public boolean hubShot() {
    return this.getRawButton(9);
  }

  public boolean launchPadShot() {
    return this.getRawButton(10);
  }

  public boolean manualIndex() {
    return this.getRawAxis(0) > 0.75;
  }

  public boolean manualOutdex() {
    return this.getRawAxis(0) < -0.75;
  }
}
