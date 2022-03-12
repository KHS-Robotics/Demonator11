// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
  public DigitalInput feederBeamBreak;
  private CANSparkMax leftSide, rightSide, feeder;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    leftSide = new CANSparkMax(RobotMap.INDEXER_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightSide = new CANSparkMax(RobotMap.INDEXER_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
    feeder = new CANSparkMax(RobotMap.INDEXER_FEEDER, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftSide.setIdleMode(IdleMode.kCoast);
    rightSide.setIdleMode(IdleMode.kCoast);

    feeder.setIdleMode(IdleMode.kBrake);
    
    rightSide.setInverted(true);

    feederBeamBreak = new DigitalInput(4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeft(double speed) {
    leftSide.setVoltage(speed * 12); //12:1
  }

  public void setRight(double speed) {
    //multiplied so that the same speed will run both motors the same
    speed *= 15/12;
    speed = MathUtil.clamp(speed, -1, 1);

    rightSide.setVoltage(speed * 12); //15:1
  }

  public void setFeeder(double speed) {
    feeder.setVoltage(speed * 12);
  }

  public void stopFeeder() {
    setFeeder(0);
  }

  public void index() {
    setLeft(0.9);
    setRight(0.9);
  }

  public void feed() {
    setFeeder(0.9);
  }

  public void stop() {
    setLeft(0);
    setRight(0);
  }
}
