// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Used to build autonomous routines.
 * @see AutonomousRoutine
 * @see AutoRoutineBuilder#build()
 * @see frc.robot.subsystems.SwerveDrive
 * @see frc.robot.subsystems.SwerveDrive#getPose()
 * @see frc.robot.subsystems.SwerveDrive#kinematics
 * @see frc.robot.subsystems.SwerveDrive#setModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[])
 * @see frc.robot.subsystems.SwerveDrive#resetNavx(Pose2d)
 */
public class AutoRoutineBuilder {
    /** The robot's trajectory config for {@link TrajectoryGenerator#generateTrajectory} */
    public TrajectoryConfig RobotTrajectoryConfig;
    /** The X-Coordinate PID Controller for the {@link CustomSwerveControllerCommand} */
    public static final PIDController SwerveXPIDController = new PIDController(1.0, 0.001, 0.2);
    /** The Y-Coordinate PID Controller for the {@link CustomSwerveControllerCommand} */
    public static final PIDController SwerveYPIDController = new PIDController(1.0, 0.001, 0.2);
    /** The Theta (rotation) PID Controller for the {@link CustomSwerveControllerCommand} */
    public static final ProfiledPIDController SwerveThetaPIDController = new ProfiledPIDController(3.5, 0.002, 0.10, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    private Pose2d startingPose;
    private ArrayList<Command> commands;

    /**
     * Creates a new <code>AutoRoutineBuilder</code>.
     * @see AutonomousRoutine
     * @see AutoRoutineBuilder#build()
     */
    public AutoRoutineBuilder(double velo, double accel) {
        this.RobotTrajectoryConfig = new TrajectoryConfig(velo, accel);
        startingPose = new Pose2d();
        commands = new ArrayList<>();
    }

    public AutoRoutineBuilder() {
        this(2.0, 3.0);
    }

    /**
     * Provides a direct way to set the starting pose for the robot.
     * @param startingPose the starting pose of the robot, AKA the location of the robot
     * at the beginning of the match relative to the top left corner of the field
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder setStartingPose(Pose2d startingPose) {
        this.startingPose = startingPose;
        return this;
    }

    /**
     * Adds a command to the autonomous routine.
     * @param command the command to add
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addCommand(Command command) {
        this.commands.add(command);
        return this;
    }

    /**
     * Adds a trajectory to follow given the trajectory JSON.
     * @param relativeFilePathToJson the file path relative to the <i>deploy</i> folder, for example <i>"./generatedPaths/trajectory.wpi.json"</i>.
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommandFromJson(String relativeFilePathToJson) {
        return this.addTrajectoryCommandFromJson(relativeFilePathToJson, false);
    }

    /**
     * Adds a trajectory to follow given the trajectory JSON.
     * @param relativeFilePathToJson the file path relative to the <i>deploy</i> folder, for example <i>"./generatedPaths/trajectory.wpi.json"</i>.
     * @param isRobotInitialPose true if startingPose is the robot's starting pose
     * on the field at the very beginning of auto, false if startingPose is <i>not</i> 
     * the robot's starting pose on the field at the very beginning of auto
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommandFromJson(String relativeFilePathToJson, boolean isRobotInitialPose) {
        Trajectory trajectory = loadPathFromJson(relativeFilePathToJson);
        if (isRobotInitialPose) {
            this.startingPose = trajectory.getInitialPose();
        }
        this.commands.add(generateSwerveCommand(trajectory));
        return this;
    }

    /**
     * Adds a trajectory for the robot to follow.
     * @param trajectory the desired trajectory for the robot to follow
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommand(Trajectory trajectory) {
        return this.addTrajectoryCommand(trajectory, false);
    }

    /**
     * Adds a trajectory for the robot to follow.
     * @param trajectory the desired trajectory for the robot to follow
     * @param isRobotInitialPose true if startingPose is the robot's starting pose
     * on the field at the very beginning of auto, false if startingPose is <i>not</i> 
     * the robot's starting pose on the field at the very beginning of auto
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommand(Trajectory trajectory, boolean isRobotInitialPose) {
        if (isRobotInitialPose) {
            this.startingPose = trajectory.getInitialPose();
        }
        this.commands.add(generateSwerveCommand(trajectory));
        return this;
    }

    /**
     * Adds a trajectory for the robot to follow.
     * @param startingPose the starting pose of the robot
     * @param endingPose the desired ending pose of the robot
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommand(Pose2d startingPose, Pose2d endingPose) {
        return addTrajectoryCommand(startingPose, endingPose, false);
    }

    /**
     * Adds a trajectory for the robot to follow.
     * @param startingPose the starting pose of the robot
     * @param endingPose the desired ending pose of the robot
     * @param isRobotInitialPose true if startingPose is the robot's starting pose
     * on the field at the very beginning of auto, false if startingPose is <i>not</i> 
     * the robot's starting pose on the field at the very beginning of auto
     * @return the AutoRoutineBuilder
     */
    public AutoRoutineBuilder addTrajectoryCommand(Pose2d startingPose, Pose2d endingPose, boolean isRobotInitialPose) {
        if (isRobotInitialPose) {
            this.startingPose = startingPose;
        }
        this.commands.add(createTrajectoryCommandFromPose(startingPose, endingPose));
        return this;
    }

    /**
     * Builds the autonomous routine.
     * @return an autonomous routine with the starting pose and associated commands.
     * @see AutonomousRoutine
     */
    public AutonomousRoutine build() {
        RobotTrajectoryConfig.setKinematics(RobotContainer.swerveDrive.kinematics);
        return new AutonomousRoutine(startingPose, new SequentialCommandGroup(this.commands.toArray(new Command[0])));
    }

    /**
     * Creates a swerve command to follow a trajectory.
     * @param trajectory the trajectory for the robot to follow
     * @return a scheduable command for the robot to follow the trajectory
     */
    public static CustomSwerveControllerCommand createTrajectoryCommand(Trajectory trajectory) {
        return generateSwerveCommand(trajectory);
    }

    /**
     * Creates a swerve command to follow a trajectory.
     * @param json the trajectory as a JSON file
     * @return a scheduable command for the robot to follow the trajectory from the JSON
     */
    public static CustomSwerveControllerCommand createTrajectoryCommandFromJson(String json) {
        return generateSwerveCommand(loadPathFromJson(json));
    }

    /**
     * Creates a swerve command to follow a trajectory.
     * @param startingPose the starting pose of the robot
     * @param endingPose the desired ending pose of the robot
     * @return a scheduable command for the robot to follow the trajectory
     */
    public CustomSwerveControllerCommand createTrajectoryCommandFromPose(Pose2d startingPose, Pose2d endingPose) {
        return generateSwerveCommand(generateTrajectory(startingPose, endingPose));
    }

    /**
     * Creates a swerve command to follow a trajectory.
     * @param startingPose the starting pose of the robot
     * @param interiorWaypoints the interior waypoints for the robot to visit before going to the ending pose
     * @param endingPose the desired ending pose of the robot
     * @return a scheduable command for the robot to follow the trajectory
     */
    public CustomSwerveControllerCommand createTrajectoryCommandFromPose(Pose2d startingPose, Translation2d[] interiorWaypoints, Pose2d endingPose) {
        return generateSwerveCommand(generateTrajectory(startingPose, interiorWaypoints, endingPose));
    }
    
    /**
     * Loads a trajectory from a JSON file relative to the <i>deploy</i> folder.
     * @param relativeFilePathToJson the file path relative to the <i>deploy</i> folder, for example <i>"./generatedPaths/trajectory.wpi.json"</i>.
     * @return the trajectory from the JSON file
     */
    private static Trajectory loadPathFromJson(String relativeFilePathToJson) {
        Trajectory trajectory;
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("./pathplanner/generatedJSON/" + relativeFilePathToJson + ".wpilib.json");
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory at path: " + relativeFilePathToJson, ex.getStackTrace());
          trajectory = new Trajectory();
        }

        return trajectory;
    }

    /**
     * Generates a swerve command using the given trajectory.
     * @param trajectory the desired trajectory for the robot to follow
     * @return a scheduable command for the robot to follow a trajectory
     */
    private static CustomSwerveControllerCommand generateSwerveCommand(Trajectory trajectory) {
        return new CustomSwerveControllerCommand(
            trajectory,
            RobotContainer.swerveDrive::getPose,
            RobotContainer.swerveDrive.kinematics,
            SwerveXPIDController,
            SwerveYPIDController,
            SwerveThetaPIDController,
            RobotContainer.swerveDrive::setModuleStates,
            RobotContainer.swerveDrive
        );
    }

    /**
     * Generates a Trajectory.
     * @param startingPose the starting pose of the robot
     * @param endingPose the desired ending pose of the robot
     * @return a usable trajectory
     */
    private Trajectory generateTrajectory(Pose2d startingPose, Pose2d endingPose) {
        return generateTrajectory(startingPose, new Translation2d[] {}, endingPose);
    }
    
    /**
     * Generates a Trajectory.
     * @param startingPose the starting pose of the robot
     * @param interiorWaypoints the interior waypoints for the robot to visit before going to the ending pose
     * @param endingPose the desired ending pose of the robot
     * @return a usable trajectory
     */
    private Trajectory generateTrajectory(Pose2d startingPose, Translation2d[] interiorWaypoints, Pose2d endingPose) {
        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();

        for (int i = 0; i < interiorWaypoints.length; i++) {
            waypoints.add(interiorWaypoints[i]);
        }

        return TrajectoryGenerator.generateTrajectory(startingPose, waypoints, endingPose, RobotTrajectoryConfig);
    }

    /**
     * Encapsulates an autonmous routine. An auto routine is composed of a
     * starting pose and a sequential command group (that may be composed of many commands).
     * @see AutoRoutineBuilder
     */
    public static class AutonomousRoutine {
        private Pose2d startingPose;
        private SequentialCommandGroup routine;

        /**
         * Creates a new <code>AutonomousRoutine</code> given a starting pose and
         * sequential command group.
         * @param startingPose the starting position of the robot relative to the top left corner of the field
         * @param routine the commands to run
         * @see AutoRoutineBuilder
         */
        public AutonomousRoutine(Pose2d startingPose, SequentialCommandGroup routine) {
            this.startingPose = startingPose;
            this.routine = routine;
        }

        /**
         * Gets the starting pose for this autonmous routine. This pose should be used
         * to reset the odometry and navx using {@link frc.robot.subsystems.SwerveDrive#resetNavx(Pose2d) resetNavx(Pose2d)}
         * in {@link Robot#autonomousInit() autonomousInit()}.
         * @return the starting position of the robot for the autonmous rotine relative to the
         * top left corner of the field along with the direction the robot is facing
         * @see Robot#autonomousInit()
         * @see frc.robot.subsystems.SwerveDrive#resetNavx(Pose2d)
         */
        public Pose2d getStartingPose() {
            return this.startingPose;
        }

        /**
         * Gets the autonomous routine as a scheduable command. This should be scheduled
         * using {@link Command#schedule() schedule()} in {@link Robot#autonomousInit() autonomousInit()}.
         * @return the autonomous routine as a scheduable command
         * @see Robot#autonomousInit()
         * @see SequentialCommandGroup#schedule()
         */
        public SequentialCommandGroup getAsCommand() {
            routine.cancel();
            CommandScheduler.getInstance().run();
            return routine;
        }
    }

    public static class CustomSwerveControllerCommand extends SwerveControllerCommand {

        public CustomSwerveControllerCommand(Trajectory trajectory, Supplier<Pose2d> pose,
                SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
                ProfiledPIDController thetaController, Consumer<SwerveModuleState[]> outputModuleStates,
                SwerveDrive swervedrive) {
            super(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, swervedrive);
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            RobotContainer.swerveDrive.stop();
        }
        
    }
}