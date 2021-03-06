// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class ManualShoot extends CommandBase {
  /**
   * Creates a new shoot.
   */
  double speed;

  public ManualShoot(double speed) {
    addRequirements(RobotContainer.shooter);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.setHood(0.5);
    RobotContainer.shooter.setShooter(speed);
    if (RobotContainer.shooter.atSetpoint(30)) {
      RobotContainer.indexer.feed();
      RobotContainer.indexer.index();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
