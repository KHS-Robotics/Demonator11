// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  private CANSparkMax motor1, motor2, motor3, motor4;
  private RelativeEncoder motor1Enc, motor2Enc, motor3Enc, motor4Enc;
  private SparkMaxPIDController motor1Pid, motor2Pid, motor3Pid, motor4Pid;

  /** Creates a new Climber. */
  public Climber() {
    motor1 = new CANSparkMax(RobotMap.CLIMBER_MOTOR1, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor2 = new CANSparkMax(RobotMap.CLIMBER_MOTOR2, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor3 = new CANSparkMax(RobotMap.CLIMBER_MOTOR3, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor4 = new CANSparkMax(RobotMap.CLIMBER_MOTOR4, CANSparkMaxLowLevel.MotorType.kBrushless);

    motor1Enc = motor1.getEncoder();
    motor2Enc = motor2.getEncoder();
    motor3Enc = motor3.getEncoder();
    motor4Enc = motor4.getEncoder();

    motor1Pid = motor1.getPIDController();
    motor2Pid = motor2.getPIDController();
    motor3Pid = motor3.getPIDController();
    motor4Pid = motor4.getPIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
