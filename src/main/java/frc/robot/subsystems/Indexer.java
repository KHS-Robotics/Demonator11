// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
  private CANSparkMax floor, side, feeder;

  /** Creates a new Indexer. */
  public Indexer() {
    floor = new CANSparkMax(RobotMap.INDEXER_FLOOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    side = new CANSparkMax(RobotMap.INDEXER_SIDE, CANSparkMaxLowLevel.MotorType.kBrushless);
    feeder = new CANSparkMax(RobotMap.INDEXER_FEEDER, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFloor (double speed) {
    floor.setVoltage(speed * 12);
  }

  public void stopFloor() {
    setFloor(0);
  }

  public void setSide (double speed) {
    side.setVoltage(speed * 12);
  }

  public void stopSide() {
    setSide(0);
  }

  public void setFeeder(double speed) {
    feeder.setVoltage(speed * 12);
  }

  public void stopFeeder() {
    setFeeder(0);
  }

  public void index() {
    setFloor(0.5);
    setSide(0.5);
  }

  public void feed() {
    setFeeder(0.5);
  }

  public void stop() {
    stopFloor();
    stopSide();
  }
}
