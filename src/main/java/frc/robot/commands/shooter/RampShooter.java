// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

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
    speed = 8.5;
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Limelight.isTarget()) {
      dist = (targetHeight - limelightHeight) / (Math.cos(Math.toRadians(Limelight.getTx())) * Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle))) + 0.91 + 0.15;

      if (dist >= 2.7) {
        angle = Math.atan(((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
      } else {
        angle = Math.atan(((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
      }

      double result = (targetHeight - robotHeight);

      speed = ridders(3.5, 13.5, angle, dist, result, 20);

      double vX = Math.cos(angle) * speed;
      double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
      double time = dist / (speed * Math.cos(angle));

      RobotContainer.shooter.setHoodAngle((Math.PI / 2) - angle);
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

  public static double ridders(double x0, double x2, double angle, double dist, double result, int iterations) {
		double x1 = (x0 + x2) / 2;

		double y0 = eq(x0, angle, dist) - result;
		double y1 = eq(x1, angle, dist) - result;
		double y2 = eq(x2, angle, dist) - result;
		
		double x3 = x1 + (x1 - x0) * Math.signum(y0)*y1/Math.sqrt((y1*y1)-(y0*y2));
		double y3 = eq(x3, angle, dist) - result;
		
		if(iterations > 0 && Math.abs(y3) > 0.01) {
			double nx0;
			
			if(y1*y3 < 0) {
				nx0 = x1;
			} else {
				if(Math.signum(x3) == Math.signum(x2)) {
					nx0 = x0;
				} else {
					nx0 = x2;
				}
			}

			if(x3 < nx0) {
				return ridders(x3, nx0, angle, dist, result, iterations - 1);
			} else {
				return ridders(nx0, x3, angle, dist, result, iterations - 1);
			}
		} else {
			return x3;
		}
	}

  public static double msToRPM(double metersPerSec) {
    //rad/s to rpm = rad/s * 30 / PI
    double r = 0.0762;
    return ((metersPerSec / (r * 2.0 / 3.0)) * 30.0 / Math.PI) * 30.0 / 24.0;
  }

  static double eq(double speed, double angle, double xDist) {
    double turn = 0;
    if (xDist == 0) {
      xDist = 0.01;
    }

    return (speed * xDist * Math.sin(angle) / ((speed * Math.cos(turn) * Math.cos(angle)))) - 9.80665 / 2 * xDist * xDist / ((2 * 0 * speed * Math.cos(turn) * Math.cos(angle)) + (speed * Math.cos(turn) * Math.cos(angle) * speed * Math.cos(turn) * Math.cos(angle)));
  }
}
