// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
  private CANSparkMax leader, follower;
  private RelativeEncoder leaderEnc;
  private SparkMaxPIDController shooterPid;

  private double shooterPidSetpoint;
  /** Creates a new Shooter. */
  public Shooter() {
    leader = new CANSparkMax(RobotMap.SHOOTER_LEADER, CANSparkMaxLowLevel.MotorType.kBrushless);
    follower = new CANSparkMax(RobotMap.SHOOTER_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);

    follower.follow(leader);
    leaderEnc = leader.getEncoder();

    shooterPid = leader.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooter(double speed) {
    shooterPidSetpoint = speed;
    shooterPid.setReference(shooterPidSetpoint, CANSparkMax.ControlType.kVelocity);
  }
}
