// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootAuto extends CommandBase {
  double dist, angle, speed;

  public ShootAuto() {
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetHeight = 2.64;
    double robotHeight = 0.6;

    dist = 0;

    if (dist >= 2.381) {
      angle = Math.atan( ((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist );
    } else {
      angle = Math.atan( ((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist );
    }

    speed = Math.sqrt(-(9.8 * dist * dist * (1 + (Math.tan(angle) * Math.tan(angle)))) / (2 * (targetHeight - robotHeight) - (2 * dist * Math.tan(angle))));

    RobotContainer.shooter.setHoodAngle(angle);
    RobotContainer.shooter.setShooter( msToRPM(speed) );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setHood(0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(RobotContainer.pixy.hasBlueInFrame() || RobotContainer.pixy.hasRedInFrame());
  }

  public static double msToRPM(double metersPerSec) {
    //rad/s to rpm = rad/s * 30 / PI
    double r = 0.0762;
    return (metersPerSec / (r * 5.0 / 6.0) ) * 30.0 / Math.PI;
  }
}