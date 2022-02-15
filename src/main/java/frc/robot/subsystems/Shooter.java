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
    shooterPidSetpoint = speed;
    shooterPid.setReference(shooterPidSetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    return Math.abs(shooterPidSetpoint - leaderEnc.getVelocity()) < 200;
  }

  public void stop() {
    leader.set(0);
  }

  static double calculateSetpointFromAngle(double angle) {
		final double topLen = 0.291; // Length from top of servo to shooter wheel
		final double bottomLen = 0.408; //Length from bottom of servo to shooter wheel

		double length = Math.sqrt( 2 * bottomLen * topLen * (-Math.cos(angle + 0.174533) + (bottomLen / topLen) ) - (bottomLen * bottomLen) + (topLen * topLen) ) ;
		double setpoint = (length - 0.1651) / 0.10627;

		return setpoint;
	}
}
