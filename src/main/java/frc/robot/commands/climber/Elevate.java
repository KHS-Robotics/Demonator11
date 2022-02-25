// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Elevate extends CommandBase {
  Level level;

  /**
   * Creates a new Elevate.
   */
  public Elevate(Level level) {
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double setpoint = 0;
    switch (level) {
      case BelowMidHeight:
        setpoint = 0;
        break;
      case MidHeight:
        setpoint = 0;
        break;
      case ClearBar:
        setpoint = 0;
        break;
      case UnderBar:
        setpoint = 0;
        break;
      case Reach:
        setpoint = 0;
        break;
      case Zero:
        setpoint = 0;
        break;
    }

    RobotContainer.climber.elevate(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.climber.elevatorAtSetpoint();
  }
}