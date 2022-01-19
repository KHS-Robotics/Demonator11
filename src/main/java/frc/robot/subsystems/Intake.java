// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private CANSparkMax driveMotor, liftMotor;
  private double speed = 0.5;
  private RelativeEncoder driveEnc;
  private RelativeEncoder liftEnc;

  private SparkMaxPIDController positionPid;

  /** Creates a new Intake. */
  public Intake() {
    driveMotor = new CANSparkMax(RobotMap.INTAKE_ROLLER, CANSparkMaxLowLevel.MotorType.kBrushless);
    liftMotor = new CANSparkMax(RobotMap.INTAKE_ANGLE, CANSparkMaxLowLevel.MotorType.kBrushless);

    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    liftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    driveEnc = driveMotor.getEncoder();
    liftEnc = liftMotor.getEncoder();

    positionPid = liftMotor.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    driveMotor.set(0);
  }
  public void intake() {
    driveMotor.set(speed);
  }
  public void reverse() {
    driveMotor.set(-speed);
  }

  public void setPosition(double angle) {
    positionPid.setReference(angle, CANSparkMax.ControlType.kPosition);
  }


}
