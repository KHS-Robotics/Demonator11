// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private CANSparkMax driveMotor, positionMotor;
  private double speed = 0.5;
  private RelativeEncoder driveEnc, positionEnc;
  private double setpoint;

  private SparkMaxPIDController positionPid;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    driveMotor = new CANSparkMax(RobotMap.INTAKE_DRIVE, MotorType.kBrushless);
    positionMotor = new CANSparkMax(RobotMap.INTAKE_POSITION, MotorType.kBrushless);

    driveMotor.setIdleMode(IdleMode.kBrake);
    positionMotor.setIdleMode(IdleMode.kBrake);

    driveEnc = driveMotor.getEncoder();
    positionEnc = positionMotor.getEncoder();

    positionPid = positionMotor.getPIDController();

    positionPid.setP(Constants.INTAKE_P);
    positionPid.setI(Constants.INTAKE_I);
    positionPid.setD(Constants.INTAKE_D);

    var tab = Shuffleboard.getTab("Intake");
    tab.addNumber("Setpoint", () -> setpoint);
    tab.addNumber("Position", positionEnc::getPosition);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    driveMotor.set(0);
  }

  public void intake() {
    driveMotor.setVoltage(12 * speed);
  }

  public void reverse() {
    driveMotor.setVoltage(12 * -speed);
  }

  public void run(double speed) {
    positionMotor.setVoltage(12 * speed);
  }

  public void setPosition(double pos) {
    setpoint = pos;
    positionPid.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public void setIdleMode(IdleMode mode) {
    positionMotor.setIdleMode(mode);
  }

  public double getPosition() {
    return positionEnc.getPosition();
  }

  public double getSpeed() {
    return driveEnc.getVelocity();
  }

  public boolean atSetpoint() {
    return Math.abs(setpoint - getPosition()) < 0.5;
  }

  public void stopPosMotor() {
    positionMotor.set(0);
  }

  public void resetPos() {
    positionEnc.setPosition(0);
  }

  public void setP(double p) {
    positionPid.setP(p);
  }
}
