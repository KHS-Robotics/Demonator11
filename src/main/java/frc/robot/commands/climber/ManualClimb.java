// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ManualClimb extends CommandBase {
  private DoubleSupplier elevatorSpeed, pivotSpeed;

  /** Creates a new ManualClimb. */
  public ManualClimb(DoubleSupplier elevatorSpeed,  DoubleSupplier pivotSpeed) {
    addRequirements(RobotContainer.climber);

    this.elevatorSpeed = elevatorSpeed;
    this.pivotSpeed = pivotSpeed;
  }

  @Override
  public void initialize() {
    RobotContainer.climber.setIdleMode(IdleMode.kBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.climber.setElevatorSpeed( Math.abs(elevatorSpeed.getAsDouble()) > 0.1 ? elevatorSpeed.getAsDouble() : 0);
    RobotContainer.climber.setPivotSpeed( Math.abs(pivotSpeed.getAsDouble()) > 0.1 ? pivotSpeed.getAsDouble() : 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setElevatorSpeed(0);
    RobotContainer.climber.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
