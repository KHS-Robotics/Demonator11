// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Pivot extends CommandBase {
  Angle angle;
  double setpoint = 0;

  public Pivot(Angle angle) {
    this.angle = angle;
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
    RobotContainer.climber.resetPivot();

    switch (angle) {
      case Rest:
        setpoint = 0;
        break;
      case Tilt:
        setpoint = -50;
        break;
      case Straight:
        setpoint = -20;
        break;
      case Handoff:
        setpoint = -39.0;
    }
  }

  @Override
  public void execute() {
    RobotContainer.climber.setAngle(setpoint);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.climber.pivotAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setPivotSpeed(0);
  }

  public enum Angle {
    Rest,
    Tilt,
    Straight,
    Handoff
  }
}
