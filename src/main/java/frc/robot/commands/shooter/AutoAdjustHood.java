// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;

public class AutoAdjustHood extends CommandBase {
  double targetHeight = Constants.TARGET_HEIGHT;
  double robotHeight = Constants.ROBOT_HEIGHT;
  double limelightHeight = Constants.LIMELIGHT_HEIGHT;
  double limelightAngle = Constants.LIMELIGHT_ANGLE;

  /** Creates a new AutoAdjustHood. */
  public AutoAdjustHood() {
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooter.setHood(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Limelight.isTarget()) {
      double dist = (targetHeight - limelightHeight) / (Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle)) * Math.cos(Math.toRadians(Limelight.getTx()))) + 0.91 + 0.15;
      double angle;

      if (dist >= 2.7) {
        angle = Math.atan(((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
      } else {
        angle = Math.atan(((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
      }

      RobotContainer.shooter.setHoodAngle( (Math.PI / 2) - angle);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
