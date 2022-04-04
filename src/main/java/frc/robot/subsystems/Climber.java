// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax elevatorLeader, elevatorFollower1, elevatorFollower2, pivotMotor;
  private RelativeEncoder elevatorEnc, pivotEnc;
  private SparkMaxPIDController elevatorPID;
  private PIDController pivotPID;

  private double offset = 0;

  private double elevatorSetpoint;
  private double pivotSetpoint;

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
    pivotPID = new PIDController(Constants.PIVOT_P, Constants.PIVOT_I, Constants.PIVOT_D);
    pivotPID.setTolerance(2);

    elevatorLeader.setIdleMode(IdleMode.kBrake);
    elevatorFollower1.setIdleMode(IdleMode.kBrake);
    elevatorFollower2.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    elevatorFollower1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    elevatorFollower2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    elevatorPID.setP(Constants.ELEVATOR_P);
    elevatorPID.setI(Constants.ELEVATOR_I);
    elevatorPID.setD(Constants.ELEVATOR_D);
    elevatorPID.setFF(0);

    elevatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kForward, true);

    elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, 59);
    elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, -5);

    elevatorPID.setOutputRange(-0.9, 0.9);

    var tab = Shuffleboard.getTab("Climb");
    tab.addNumber("Pitch!", () -> getPitch());
    tab.addNumber("offset", () -> offset);
    tab.addNumber("Roll", RobotContainer.navx::getRoll);
    tab.addNumber("Elevator Position", elevatorEnc::getPosition);
    tab.addNumber("Pivot Position", pivotEnc::getPosition);
    tab.addNumber("El current", () -> RobotContainer.pdp.getCurrent(RobotMap.ELEVATOR_LEADER));
    tab.addNumber("Pivot Current", () -> RobotContainer.pdp.getCurrent(RobotMap.PIVOT_MOTOR));
    tab.addNumber("Error", pivotPID::getPositionError);
    tab.addNumber("Pivot Speed", () -> pivotSetpoint);
  }

  public void setAngle(double angle) {
    setPivotSpeed( -MathUtil.clamp( pivotPID.calculate(getPitch(), angle), -0.5, 0.5));
  }

  public void elevate(double height) {
    elevatorSetpoint = height;

    elevatorPID.setReference(height, ControlType.kPosition);
  }

  public void resetPitch() {
    offset = -RobotContainer.navx.getRoll();
  }

  public double getPitch() {
    return RobotContainer.navx.getRoll() + offset;
  }

  public boolean elevatorAtSetpoint() {
    return Math.abs(elevatorSetpoint - elevatorEnc.getPosition()) < 0.5;
  }

  public boolean pivotAtSetpoint() {
    return pivotPID.atSetpoint();
  }

  public void resetPivot() {
    pivotPID.reset();
  }

  public void stopPosMotor() {
    pivotMotor.set(0);
    pivotPID.reset();
  }

  public void setElevatorSpeed(double speed) {
    elevatorLeader.setVoltage(12 * speed);
  }

  public void setIdleMode(IdleMode mode) {
    elevatorLeader.setIdleMode(mode);
    elevatorFollower1.setIdleMode(mode);
    elevatorFollower2.setIdleMode(mode);
  }

  public void setPivotSpeed(double speed) {
    pivotSetpoint = speed;
    pivotMotor.setVoltage(12 * speed);
  }

  public void resetPos() {
    elevatorEnc.setPosition(0);
    pivotPID.reset();
  }

  public boolean softLimitsOn() {
    return elevatorLeader.isSoftLimitEnabled(SoftLimitDirection.kForward) && elevatorLeader.isSoftLimitEnabled(SoftLimitDirection.kReverse);
  }

  public void setSoftLimits(boolean enable) {
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kForward, enable);
    elevatorLeader.enableSoftLimit(SoftLimitDirection.kReverse, enable);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
