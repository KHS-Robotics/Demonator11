// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Indexer extends SubsystemBase {
  private CANSparkMax motor1, motor2;

  /** Creates a new Indexer. */
  public Indexer() {
    motor1 = new CANSparkMax(RobotMap.INDEXER_MOTOR1, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkMax(RobotMap.INDEXER_MOTOR2, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor1 (double speed) {
    motor1.set(speed);
  }

  public void stopMotor1() {
    setMotor1(0);
  }

  public void setMotor2 (double speed) {
    motor2.set(speed);
  }

  public void stopMotor2() {
    setMotor2(0);
  }
}
