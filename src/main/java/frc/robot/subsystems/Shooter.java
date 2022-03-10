// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private CANSparkMax leader, follower;
  private RelativeEncoder leaderEnc;
  private SparkMaxPIDController shooterPid;
  private Servo hoodServo1, hoodServo2;

  private double shooterPidSetpoint;
  private double speedMultiplier;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER_LEADER, MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, MotorType.kBrushless);

    follower.follow(leader, true);
    leaderEnc = leader.getEncoder();

    shooterPid = leader.getPIDController();
    shooterPid.setOutputRange(0, 1);

    leaderEnc.setVelocityConversionFactor(1);
    leader.setIdleMode(IdleMode.kCoast);
    follower.setIdleMode(IdleMode.kCoast);

    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    leader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    follower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    
    shooterPid.setP(Constants.SHOOTER_P);
    shooterPid.setI(Constants.SHOOTER_I);
    shooterPid.setD(Constants.SHOOTER_D);
    shooterPid.setFF(Constants.SHOOTER_FF);

    hoodServo1 = new Servo(RobotMap.HOOD_SERVO_1);
    hoodServo2 = new Servo(RobotMap.HOOD_SERVO_2);

    hoodServo1.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    hoodServo2.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    speedMultiplier = 1.05;

    var tab = Shuffleboard.getTab("Match");
    tab.addNumber("Shooter Multiplier", () -> speedMultiplier);

    tab = Shuffleboard.getTab("Shooter");
    tab.addNumber("Speed", leaderEnc::getVelocity);
    tab.addNumber("Error", () -> shooterPidSetpoint - leaderEnc.getVelocity() );
    tab.addNumber("Shooter Multiplier", () -> speedMultiplier);
    tab.addNumber("Setpoint", () -> shooterPidSetpoint);
  }

  public void setHoodAngle(double angle) {
    double setpoint = calculateSetpointFromAngle(angle);

    hoodServo1.set(setpoint);
    hoodServo2.set(setpoint);
  }

  public void setHood(double setpoint) {
    hoodServo1.set(setpoint);
    hoodServo2.set(setpoint);
  }

  public void setShooter(double speed) {
    shooterPidSetpoint = speed * speedMultiplier;
    shooterPid.setReference(shooterPidSetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    return Math.abs(shooterPidSetpoint - leaderEnc.getVelocity()) < 30;
  }

  public void stop() {
    leader.set(0);
  }

  static double calculateSetpointFromAngle(double angle) {
    final double topLen = 0.291; // Length from top of servo to shooter wheel
    final double bottomLen = 0.408; //Length from bottom of servo to shooter wheel

    double length = Math.sqrt(2 * bottomLen * topLen * (-Math.cos(angle + 0.174533) + (bottomLen / topLen)) - (bottomLen * bottomLen) + (topLen * topLen));
    double setpoint = (length - 0.1651) / 0.10627;

    return setpoint;
  }

  public void incrementMultiplier() {
    if (speedMultiplier < 1.3) {
      speedMultiplier += 0.02;
    }
  }

  public void decrementMultiplier() {
    if (speedMultiplier > 0.8) {
      speedMultiplier -= 0.02;
    }
  }

  public void resetMultipler() {
    speedMultiplier = 1.05;
  }
}
