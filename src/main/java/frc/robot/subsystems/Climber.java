// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax elevatorLeader, elevatorFollower1, elevatorFollower2, pivotMotor;
  private RelativeEncoder elevatorEnc, pivotEnc;
  private SparkMaxPIDController elevatorPID;
  private PIDController anglePID;

  private double pivotSetpoint, elevatorSetpoint;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    elevatorLeader = new CANSparkMax(RobotMap.ELEVATOR_LEADER, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower1 = new CANSparkMax(RobotMap.ELEVATOR_FOLLOWER1, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorFollower2 = new CANSparkMax(RobotMap.ELEVATOR_FOLLOWER2, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorFollower1.follow(elevatorLeader, true);
    elevatorFollower2.follow(elevatorLeader, true);

    pivotMotor = new CANSparkMax(RobotMap.PIVOT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorEnc = elevatorLeader.getEncoder();
    pivotEnc = pivotMotor.getEncoder();

    elevatorPID = elevatorLeader.getPIDController();
    anglePID = new PIDController(0.075, 0, 0);
    anglePID.setTolerance(2);

    elevatorLeader.setIdleMode(IdleMode.kBrake);
    elevatorFollower1.setIdleMode(IdleMode.kBrake);
    elevatorFollower2.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    elevatorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    elevatorLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    
    elevatorEnc.setPosition(0);

    elevatorPID.setP(0.1);
    elevatorPID.setI(0.0001);
    elevatorPID.setD(0);
    elevatorPID.setFF(0);

    elevatorPID.setOutputRange(-0.75, 1);

    var tab = Shuffleboard.getTab("Climb");
    tab.addNumber("Pitch!", () -> RobotContainer.navx.getRoll());
    tab.addNumber("Elevator Position", elevatorEnc::getPosition);
    tab.addNumber("Pivot Position", pivotEnc::getPosition);
    tab.addNumber("El current", () -> RobotContainer.pdp.getCurrent(RobotMap.ELEVATOR_LEADER));
    tab.addNumber("Pivot Current", () -> RobotContainer.pdp.getCurrent(RobotMap.PIVOT_MOTOR));
    tab.addNumber("Error", anglePID::getPositionError);
  }

  public void setAngle(double angle) {
    pivotMotor.set( MathUtil.clamp( anglePID.calculate(RobotContainer.navx.getRoll(), angle), -1, 1));
  }

  public void elevate(double height) {
    elevatorSetpoint = height;

    elevatorPID.setReference(height, ControlType.kPosition);
  }

  public boolean elevatorAtSetpoint() {
    return Math.abs(elevatorSetpoint - elevatorEnc.getPosition()) < 0.5;
  }

  public boolean pivotAtSetpoint() {
    return Math.abs(pivotSetpoint - pivotEnc.getPosition()) < 0.5;
  }

  public void stopPosMotor() {
    pivotMotor.set(0);
    anglePID.reset();
  }

  public void setElevatorSpeed(double speed) {
    elevatorLeader.setVoltage(12 * speed);
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.setVoltage(12 * speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
