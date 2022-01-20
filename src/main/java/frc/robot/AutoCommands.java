/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.drive.rotate.RotateToAngle;

/**
 * Class to generate commands for autonomous movement
 */
public class AutoCommands {
  private static boolean initialized = false;
  public static SwerveControllerCommand threeBall, threeBallA, threeBallB;
  public static SwerveControllerCommand twoBall;
  public static TrajectoryConfig config = new TrajectoryConfig(3.0, 6.0);

  public static void autoInit() {
    if (!initialized) {
      new Thread(() -> {

        config.setKinematics(RobotContainer.swerveDrive.kinematics);

        twoBall = generatePath(new Pose2d[] {
            new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)),
            new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(270))
        });

        threeBall = generatePath(new Pose2d[] {
            new Pose2d(7.4, 2.0, Rotation2d.fromDegrees(55)),
            new Pose2d(5.0, 1.8, Rotation2d.fromDegrees(180)),
            new Pose2d(5.4, 1.8, Rotation2d.fromDegrees(180)),
            new Pose2d(6.0, 1.0, Rotation2d.fromDegrees(315)),
            new Pose2d(7.2, 0.45, Rotation2d.fromDegrees(315)),
            new Pose2d(7.5, 1.5, Rotation2d.fromDegrees(90))
        });

        threeBallA = generatePath(new Pose2d[] {
            new Pose2d(7.4, 2.0, Rotation2d.fromDegrees(55)),
            new Pose2d(5.0, 1.8, Rotation2d.fromDegrees(180)),
            new Pose2d(5.4, 1.8, Rotation2d.fromDegrees(180))
        });

        threeBallB = generatePath(new Pose2d[] {
            // new Pose2d(5.4, 1.8, Rotation2d.fromDegrees(180)), // <- NOT SURE IF NEEDED, TEST
            new Pose2d(6.0, 1.0, Rotation2d.fromDegrees(315)),
            new Pose2d(7.2, 0.45, Rotation2d.fromDegrees(315)),
        });

      }).start();

      initialized = true;
    }
  }

  public static SwerveControllerCommand generatePath(Pose2d start, Pose2d end) {
    return generatePath(start, new Translation2d[] {}, end);
  }

  public static SwerveControllerCommand generatepath(Pose2d[] points) {
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

    for (int i = 0; i < points.length; i++) {
      waypoints.add(points[i]);
    }

    return getCommandFromTrajectory(TrajectoryGenerator.generateTrajectory(waypoints, config));
  }

  public static SwerveControllerCommand generatePath(Pose2d start, Translation2d[] mid, Pose2d end) {
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();

    for (int i = 0; i < mid.length; i++) {
      interiorWaypoints.add(mid[i]);
    }

    return getCommandFromTrajectory(TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config));
  }

  public static SwerveControllerCommand generatePath(Pose2d[] points) {
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

    for (int i = 0; i < points.length; i++) {
      waypoints.add(points[i]);
    }

    return getCommandFromTrajectory(TrajectoryGenerator.generateTrajectory(waypoints, config));
  }

  public static SwerveControllerCommand getCommandFromTrajectory(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta (rotation)
        RobotContainer.swerveDrive::setModuleStates, RobotContainer.swerveDrive);
  }

  public static SwerveControllerCommand loadPathFromJSON(String json) {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + json + ".wpilib.json");
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + json, ex.getStackTrace());
      return null;
    }

    return new SwerveControllerCommand(
        trajectory,
        RobotContainer.swerveDrive::getPose,
        RobotContainer.swerveDrive.kinematics,
        // Position controllers
        new PIDController(0.90, 0.001, 0.30), // x (forward/backwards)
        new PIDController(0.90, 0.001, 0.30), // y (side to side)
        new ProfiledPIDController(1.75, 0.001, 0.40, new TrapezoidProfile.Constraints(Math.PI, Math.PI)), // theta
                                                                                                          // (rotation)
        RobotContainer.swerveDrive::setModuleStates,
        RobotContainer.swerveDrive);
  }

  public static Command twoBall() {
    Command shoot = new InstantCommand();
    Command checkForIntake = new InstantCommand();

    return (twoBall.raceWith(checkForIntake)).andThen(new RotateToAngle(90)).andThen(shoot);
  }

  public static Command threeBall() {
    Command shoot = new InstantCommand();
    return shoot.andThen(threeBall).andThen(shoot);
  }

  public static Command threeBallRace() {
    Command shoot = new InstantCommand();
    Command checkForIntake = new InstantCommand();

    return shoot.andThen(threeBallA.raceWith(checkForIntake)).andThen(threeBallB.raceWith(checkForIntake))
        .andThen(new RotateToAngle(90)).andThen(shoot);
  }
}