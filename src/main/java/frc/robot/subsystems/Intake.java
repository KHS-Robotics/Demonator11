// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private CANSparkMax driveMotor, positionMotor;
  private double speed = 0.3;
  private RelativeEncoder driveEnc, positionEnc;

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
    positionPid.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public void intakeUp() {
    setPosition(0);
    positionMotor.setIdleMode(IdleMode.kBrake);
  }

  public void intakeDown() {
    setPosition(0);
    positionMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getPosition() {
    return positionEnc.getPosition();
  }

  public double getSpeed() {
    return driveEnc.getVelocity();
  }

}
