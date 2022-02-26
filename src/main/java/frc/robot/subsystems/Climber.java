// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax elevatorLeader, elevatorFollower1, elevatorFollower2, pivotMotor;
  private RelativeEncoder elevatorEnc, pivotEnc;
  private SparkMaxPIDController elevatorPID, pivotPID;

  private double pivotSetpoint, elevatorSetpoint;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    elevatorLeader = new CANSparkMax(RobotMap.ELEVATOR_LEADER, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower1 = new CANSparkMax(RobotMap.ELEVATOR_FOLLOWER1, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower2 = new CANSparkMax(RobotMap.ELEVATOR_FOLLOWER2, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorFollower1.follow(elevatorLeader);
    elevatorFollower2.follow(elevatorLeader);

    pivotMotor = new CANSparkMax(RobotMap.PIVOT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorEnc = elevatorLeader.getEncoder();
    pivotEnc = pivotMotor.getEncoder();

    elevatorPID = elevatorLeader.getPIDController();
    pivotPID = pivotMotor.getPIDController();
  }

  public void elevate(double height) {
    elevatorSetpoint = height;

    elevatorPID.setReference(height, ControlType.kPosition);
  }

  public boolean elevatorAtSetpoint() {
    return Math.abs(elevatorSetpoint - elevatorEnc.getPosition()) < 0.5;
  }

  public void pivot(double angle) {
    pivotSetpoint = angle;

    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public boolean pivotAtSetpoint() {
    return Math.abs(pivotSetpoint - pivotEnc.getPosition()) < 0.5;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
