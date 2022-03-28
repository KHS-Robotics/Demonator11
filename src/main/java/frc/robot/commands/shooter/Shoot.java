// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class Shoot extends CommandBase {

  double dist, angle, speed, lastVelocity, lastAccel, lastJerk;

  double targetHeight = Constants.TARGET_HEIGHT;
  double robotHeight = Constants.ROBOT_HEIGHT;
  double limelightHeight = Constants.LIMELIGHT_HEIGHT;
  double limelightAngle = Constants.LIMELIGHT_ANGLE;

  public Shoot() {
    addRequirements(RobotContainer.shooter, RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 8.5;
    Limelight.setLedMode(LightMode.eOn);

    dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle)) + 0.91 + 0.15;


    System.out.println("START");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Limelight.isTarget()) {
      dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle)) + 0.91 + 0.15;
    }

    if (dist > 2.7) {
      angle = Math.atan(((Math.tan(-0.698131701) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
    } else {
      angle = Math.atan(((Math.tan(-1.21) * (dist)) - (2 * (targetHeight - robotHeight))) / -dist);
    }
    
    RobotContainer.shooter.setHoodAngle((Math.PI / 2) - angle);

    double result = (targetHeight - robotHeight);
   
    speed = ridders(3.5, 13.5, angle, dist, result, 20);

    double vX = Math.cos(angle) * speed;
    double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
    double time = dist / (speed * Math.cos(angle));

    double maxError = ridders(3.5, 13.5, angle, dist + 0.2, result, 20);

    double vXMax = Math.cos(angle) * maxError;
    double initDragMax = 0.2 * 1.225 * 0.0145564225 * Math.PI * vXMax * vXMax / 0.27;
    double timeMax = (dist+0.308) / ( maxError * Math.cos(angle) );

    maxError += (initDragMax * timeMax * timeMax * 0.5);

    double minError = ridders(3.5, 13.5, angle, dist - 0.2, result, 20);

    double vXMin = Math.cos(angle) * minError;
    double initDragMin = 0.2 * 1.225 * 0.0145564225 * Math.PI * vXMin * vXMin / 0.27;
    double timeMin = (dist+0.308) / ( minError * Math.cos(angle) );

    minError += (initDragMin * timeMin * timeMin * 0.5);

    RobotContainer.shooter.setShooter(msToRPM(speed + (initDrag * time * time * 0.5)));

    RobotContainer.indexer.index();



    double shooterAccel = (RobotContainer.shooter.getVelocity() - lastVelocity) * 50;
    double shooterJerk = shooterAccel - lastAccel;
    double shooterSnap = shooterJerk - lastJerk;
    lastVelocity = RobotContainer.shooter.getVelocity();
    lastAccel = shooterAccel;
    lastJerk = shooterJerk;



      if (nextLeave(shooterAccel, shooterJerk, shooterSnap, minError, maxError) > 0.3 && nextEnter(shooterAccel, shooterJerk, shooterSnap, minError, maxError) < 0.01) {
        RobotContainer.indexer.feed();
    } else {
      RobotContainer.indexer.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.stop();
    RobotContainer.shooter.setHood(0.5);
    RobotContainer.indexer.stopFeeder();
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

    /**
     *
     * @return the time the shooter's rpm will be at offset during t0<t<t2
     */
    public static double cubicSolve(double t0, double t2, double accel, double jerk, double snap, double offset, int iterations) {
        double t1 = (t0 + t2) / 2;

        double y0 = taylorRPM(accel, jerk, snap, t0) + offset;
        double y1 = taylorRPM(accel, jerk, snap, t1) + offset;
        double y2 = taylorRPM(accel, jerk, snap, t2) + offset;

        double t3 = t1 + (t1 - t0) * Math.signum(y0)*y1/Math.sqrt((y1*y1)-(y0*y2));
        double y3 = taylorRPM(accel, jerk, snap, t3) + offset;

        if(iterations > 0 && Math.abs(y3) > 0.01) {
            double nx0;

            if(y1*y3 < 0) {
                nx0 = t1;
            } else {
                if(Math.signum(t3) == Math.signum(t2)) {
                    nx0 = t0;
                } else {
                    nx0 = t2;
                }
            }

            if(t3 < nx0) {
                return cubicSolve(t3, nx0, accel, jerk, snap, offset, iterations - 1);
            } else {
                return cubicSolve(nx0, t3, accel, jerk, snap, offset, iterations - 1);
            }
        } else {
            return t3;
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

    /**
     *
     * @return cubic taylor approximation of rpm at time t in seconds
     */
  private static double taylorRPM(double accel, double jerk, double snap, double t) {
      return RobotContainer.shooter.getVelocity() + accel * t + jerk * Math.pow(t, 2) / 2 + snap * Math.pow(t, 3) / 6;
  }

    /**
     *
     * @return location of the next extreme of the cubic approximation of rpm as t in seconds, or 0 if the rpm is not approaching the setpoint
     */
  private static double nextPeak(double accel, double jerk, double snap) {
      double c = accel;
      double b = jerk;
      double a = snap / 2;
      if(Math.pow(b, 2) - 4 * a * c < 0) {
        return (-b + Math.pow(b, 2) - 4 * a * c) / (2 * a);
      }
      return 0;
  }

    /**
     *
     * @return will return the next t the rpm will enter the desired error, or 0 if within allowed error
     */
  public static double nextEnter(double accel, double jerk, double snap, double minError, double maxError) {
      if (RobotContainer.shooter.getVelocity() > maxError) {
          return cubicSolve(0, nextPeak(accel, jerk, snap), accel, jerk, snap, maxError, 20);
      } else if (RobotContainer.shooter.getVelocity() < minError) {
          return cubicSolve(0, nextPeak(accel, jerk, snap), accel, jerk, snap, minError, 20);
      }
      return 0;
  }
    /**
     *
     * @return will return the next t the rpm will leave the desired error, or -1 if it cannot find a solution
     * (this should never happen, due to the error in the approximation it should become inaccurate after 2 extremes and leave the allowed rpm error)
     */
    public static double nextLeave(double accel, double jerk, double snap, double minError, double maxError) {
        if (RobotContainer.shooter.getVelocity() < maxError) {
            return cubicSolve(0, nextPeak(accel, jerk, snap), accel, jerk, snap, maxError, 20);
        } else if (RobotContainer.shooter.getVelocity() > minError) {
            return cubicSolve(0, nextPeak(accel, jerk, snap), accel, jerk, snap, minError, 20);
        }
        return -1;
    }
}
