// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class PrepElevator extends CommandBase {
  boolean finished = false;
  Timer timer = new Timer();

  /** Creates a new PrepElevator. */
  public PrepElevator() {
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    RobotContainer.climber.setElevatorSpeed(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finished = RobotContainer.pdp.getCurrent(RobotMap.ELEVATOR_LEADER) > 2.5;
    if(!finished) {
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished && timer.hasElapsed(0.06);
  }
}
