// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LEDStrip extends SubsystemBase {
  public final int port, length;
  
  private int strobeIndex = 0;
  private Mode mode;
  private Color color;
  private AddressableLED strip;
  private AddressableLEDBuffer buffer;

  /** Creates a new LEDStrip. */
  public LEDStrip(int pwmPort, int length) {
    this.port = pwmPort;
    this.length = length;

    strip = new AddressableLED(port);
  }

  public void init() {
    buffer = new AddressableLEDBuffer(length);
    strip.setLength(buffer.getLength());

    // Set the data
    strip.setData(buffer);
    strip.start();
  }

  public void setLEDMode(Mode mode) {
    this.mode = mode;

    if(mode == Mode.kStobe) {
      strobeIndex = 0;
    }
  }

  public void setColor(ColorMode colorMode) {
    if (colorMode == ColorMode.kTeam) {
      colorMode = Robot.color == Alliance.Red ? ColorMode.kRed : ColorMode.kBlue;
    }

    switch (colorMode) {
      case kRed:
        color = new Color(237, 28, 36);
        break;
      case kBlue:
        color = new Color(0, 102, 179);
        break;
      case kGreen:
        color = new Color(0, 150, 55);
        break;
      default:
        color = new Color(0, 102, 179);
        break;
    }
  }

  public void setColor(int r, int g, int b) {
    this.color = new Color(r, g, b);

    for(int i = 0; i < length; i++) {
      buffer.setRGB(i, color.r, color.g, color.b);
    }

    strip.setData(buffer);
  }

  @Override
  public void periodic() {
    if (mode == Mode.kStobe) {
      for (int i = 0; i < length; i++) {
        if(Math.abs(i - strobeIndex) < 20 ) {
          buffer.setRGB(i, 0, 0, 0);
        } else {
          buffer.setRGB(i, color.r, color.g, color.b);
        }
      }

      strip.setData(buffer);
    }
  }

  public enum ColorMode {
    kBlue,
    kRed,
    kGreen,
    kTeam
  }

  public enum Mode {
    kSolid,
    kStobe
  }

  public class Color {
    final int r, g, b;

    public Color(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }
}