// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private CANSparkMax leader, follower;
  private RelativeEncoder leaderEnc;
  private SparkMaxPIDController shooterPid;
  private Servo hoodServo1, hoodServo2;

  private double shooterPidSetpoint;
  /** Creates a new Shooter. */
  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER_LEADER, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, MotorType.kBrushless);

    follower.follow(leader);
    leaderEnc = leader.getEncoder();

    shooterPid = leader.getPIDController();
  
    hoodServo1 = new Servo(0);
    hoodServo2 = new Servo(1);

    hoodServo1.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    hoodServo2.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setHoodAngle(double angle) {
    double setpoint = angle;

    hoodServo1.set(setpoint);
    hoodServo2.set(setpoint);
  }

  public void setShooter(double speed) {
    shooterPidSetpoint = speed;
    shooterPid.setReference(shooterPidSetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    return Math.abs(shooterPidSetpoint - leaderEnc.getVelocity()) < 200;
  }

  public void stop() {
    leader.set(0);
  }
}
