// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.Pivot.Angle;

public class WaitForNavx extends CommandBase {
  Timer timer = new Timer();
  Angle angle;
  boolean atSetpoint;

  /**
   * Creates a new WaitForNavx.
   */
  public WaitForNavx(Angle angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
  }

  @Override
  public void initialize() {
    timer.start();
  }

  @Override
  public void execute() {
    double pitch = 0;

    switch (angle) {
      case Rest:
        pitch = 0;
        break;
      case Tilt:
        pitch = 0;
        break;
      case Straight:
        pitch = 0;
        break;
      case Handoff:
        pitch = 0;
    }

    atSetpoint = Math.abs(RobotContainer.navx.getPitch() - pitch) < 10;
    if (!atSetpoint) {
      timer.reset();
    } else {
      atSetpoint = timer.hasElapsed(0.5);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint;
  }
}
