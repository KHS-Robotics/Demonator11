// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.RobotContainer;

public class ShootAuto extends Shoot {
  public ShootAuto() {
    super();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setHood(0.5);
    RobotContainer.indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(RobotContainer.pixy.hasBlueInFrame() || RobotContainer.pixy.hasRedInFrame());
  }
}