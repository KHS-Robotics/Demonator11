// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax elevatorLeader, elevatorFollower1, elevatorFollower2, pivotMotor;
  private RelativeEncoder elevatorEnc, pivotEnc;
  private SparkMaxPIDController elevatorPID, pivotPID;
  double elevateSetpoint;

  /** Creates a new Climber. */
  public Climber() {
    elevatorLeader = new CANSparkMax(RobotMap.CLIMBER_MOTOR1, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower1 = new CANSparkMax(RobotMap.CLIMBER_MOTOR2, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower2 = new CANSparkMax(RobotMap.CLIMBER_MOTOR3, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    elevatorFollower1.follow(elevatorLeader);
    elevatorFollower2.follow(elevatorLeader);
    
    pivotMotor = new CANSparkMax(RobotMap.CLIMBER_MOTOR4, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorEnc = elevatorLeader.getEncoder();
    pivotEnc = pivotMotor.getEncoder();

    elevatorPID = elevatorLeader.getPIDController();
    pivotPID = pivotMotor.getPIDController();
  }

  public void elevate(double height) {
    elevateSetpoint = height;
    elevatorPID.setReference(height, ControlType.kPosition);
  }
  public boolean elevatorAtSetpoint( ){
    return Math.abs(elevateSetpoint - elevatorEnc.getPosition()) < 0.5;
  }
  public void pivot(double angle) {
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
