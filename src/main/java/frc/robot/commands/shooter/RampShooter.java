// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;

public class RampShooter extends CommandBase {
  double dist, angle, speed;

  double targetHeight = Constants.TARGET_HEIGHT;
  double robotHeight = Constants.ROBOT_HEIGHT;
  double limelightHeight = Constants.LIMELIGHT_HEIGHT;
  double limelightAngle = Constants.LIMELIGHT_ANGLE;

  /**
   * Creates a new RampShooter.
   */
  public RampShooter() {
    addRequirements(RobotContainer.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 8;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle));

    if (dist >= 2.7) {
      angle = Math.atan(((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
    } else {
      angle = Math.atan(((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
    }

    double result = (targetHeight - robotHeight);
    double error = result - eq(speed, angle, dist);

    for (int i = 0; i < 40; i++) {
      if (Math.abs(error) > 0.2) {
        if (error > 0) {
          speed += speed / 2;
        } else {
          speed -= speed / 2;
        }
        error = result - eq(speed, angle, dist);
      } else {
        break;
      }
    }

    double vX = Math.cos(angle) * speed;
    double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
    double time = dist / (speed * Math.cos(angle));

    RobotContainer.shooter.setHoodAngle((Math.PI / 2) - angle);

    if (Math.abs(error) < 0.5) {
      RobotContainer.shooter.setShooter(msToRPM(speed + (initDrag * time * time * 0.5)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static double msToRPM(double metersPerSec) {
    //rad/s to rpm = rad/s * 30 / PI
    double r = 0.0762;
    return (metersPerSec / (r * 2.0 / 3.0)) * 30.0 / Math.PI;
  }

  static double eq(double speed, double angle, double xDist) {
    double turn = 0;
    if (xDist == 0) {
      xDist = 0.01;
    }

    return (speed * xDist * Math.sin(angle) / ((speed * Math.cos(turn) * Math.cos(angle)))) - 9.80665 / 2 * xDist * xDist / ((2 * 0 * speed * Math.cos(turn) * Math.cos(angle)) + (speed * Math.cos(turn) * Math.cos(angle) * speed * Math.cos(turn) * Math.cos(angle)));
  }
}