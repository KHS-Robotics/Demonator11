/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoRoutineBuilder.AutonomousRoutine;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.climber.*;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToAngle;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAuto;
import frc.robot.subsystems.*;
import frc.robot.vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static NetworkTableEntry id;
  public static final AHRS navx = new AHRS(Port.kUSB);

  public static final PixyCam pixy = new PixyCam();

  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Indexer indexer = new Indexer();
  public static final Climber climber = new Climber();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());

    var tab = Shuffleboard.getTab("Match");
    tab.addNumber("X", xboxController::getLeftX);
    tab.addNumber("Y", xboxController::getLeftY);
    id = tab.add("AUTO ID", 0).getEntry();
    tab.addNumber("dist", () -> (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.getTy() + Constants.LIMELIGHT_ANGLE)));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button autoCalibrateTeleop = new Button(() -> (!swerveDrive.isCalibrated && RobotState.isTeleop() && RobotState.isEnabled()));
    autoCalibrateTeleop.whenPressed(new CenterSwerveModules(true));

    JoystickButton forceCalibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    forceCalibrate.whenPressed(new CenterSwerveModules(true));

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    Button holdAngle = new Button(xboxController::getAButton);
    holdAngle.whenHeld(new HoldAngleWhileDriving());

    Button resetNavxButton = new Button(xboxController::getStartButton);
    resetNavxButton.whenPressed(() -> swerveDrive.resetNavx(), swerveDrive);

    Button slowDrive = new Button(() -> (xboxController.getLeftTriggerAxis() > 0.3));
    slowDrive.whenPressed(() -> {
      SwerveDrive.kMaxAngularSpeed = Math.PI / 2;
      SwerveDrive.kMaxSpeed = 2;
    });
    slowDrive.whenReleased(() -> {
      SwerveDrive.kMaxAngularSpeed = Math.PI;
      SwerveDrive.kMaxSpeed = 3.5;
    });

    Button intakeBall = new Button(switchbox::intake);
    intakeBall.whileHeld(() -> intake.intake(), intake);
    intakeBall.whenReleased(() -> intake.stop(), intake);

    Button outtakeBall = new Button(switchbox::outtake);
    outtakeBall.whileHeld(() -> {intake.reverse(); indexer.reverse();}, intake);
    outtakeBall.whenReleased(() -> intake.stop(), intake);

    Button shoot = new Button(switchbox::shoot);
    shoot.whenHeld(new Shoot());

    Button manualIndex = new Button(() -> false);
    manualIndex.whenPressed(() -> indexer.index());
    manualIndex.whenReleased(() -> indexer.stop());

    Button climb = new Button(() -> false);
    climb.whenPressed(
      new SequentialCommandGroup(
        new Elevate(Level.BelowMidHeight),
        new Elevate(Level.MidHeight),
        new Elevate(Level.Zero),
        new Elevate(Level.ClearBar),
        new Pivot(Angle.Tilt)
          .deadlineWith(new Elevate(Level.UnderBar)),
        new WaitForNavx(Angle.Tilt),
        new Elevate(Level.Reach)
      )
    );
  }

  public static AutonomousRoutine getCommand(int id) {

    switch (id) {
      default:
        return new AutonomousRoutine(new Pose2d(), new SequentialCommandGroup());
      case 0:
        return getTwoBallAuto();
      case 1:
        return getThreeBallClose();
      case 2:
        return getThreeBallFar();
      case 3:
        return getThreeBallMid();
      case 4:
        return getFourBallAuto();
      case 5:
        return getFiveBallAuto();
    }
  }

  private static AutonomousRoutine getTwoBallAuto() {
    return new AutoRoutineBuilder()
      .addTrajectoryCommand(
        new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)),
        new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(270)),
        true
      ).addCommand(
        new RotateToAngle(90)
      ).addCommand(
        new ShootAuto()
      ).build();
  }

  private static AutonomousRoutine getThreeBallClose() {
    return new AutoRoutineBuilder()
      .addCommand(
        new ShootAuto()
      )
      .addTrajectoryCommand(
        new Pose2d(7.6, 1.80, Rotation2d.fromDegrees(90)),
        new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(180)),
        true
      ).addCommand(
        new RotateToAngle(318.27, 10)
      ).addTrajectoryCommand(
        new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(318.27)),
        new Pose2d(7.3, 0.3, Rotation2d.fromDegrees(318.27))
      ).addCommand(
        new RotateToAngle(90)
      ).addCommand(
        new ShootAuto()
      )
      .build();
  }

  private static AutonomousRoutine getThreeBallMid() {
    return new AutoRoutineBuilder(1.5, 2)
      .addCommand(
        new ShootAuto()
      ).addCommand(
        new RotateToAngle(270, 15)
      ).addTrajectoryCommand(
        new Pose2d(7.6, 1.80, Rotation2d.fromDegrees(90)),
        new Pose2d(7.5, 0.3, Rotation2d.fromDegrees(270)),
        true
      ).addCommand(
        new RotateToAngle(140, 15)
      ).addTrajectoryCommand(
        new Pose2d(7.5, 0.3, Rotation2d.fromDegrees(140)),
        new Pose2d(5.5, 1.6, Rotation2d.fromDegrees(140))
      ).addCommand(
        new RotateToAngle(50, 10)
      ).addCommand(
        new ShootAuto()
      ).build();
  }

  private static AutonomousRoutine getThreeBallFar() {
    return new AutoRoutineBuilder()
      .setStartingPose(
        new Pose2d(6.6, 2.56, Rotation2d.fromDegrees(45))
      ).addCommand(
        new ShootAuto()
      ).addCommand(
        new RotateToAngle(-170, 10)
      ).addTrajectoryCommand(
        new Pose2d(6.6, 2.56, Rotation2d.fromDegrees(-170)),
        new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(-170))
      ).addTrajectoryCommand(
        new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(-170)),
        new Pose2d(1.65, 1.30, Rotation2d.fromDegrees(-170))
      ).addCommand(
        new RotateToAngle(30, 10)
      ).addTrajectoryCommand(
        new Pose2d(1.65, 1.30, Rotation2d.fromDegrees(30)),
        new Pose2d(5.1, 1.4, Rotation2d.fromDegrees(30))
      ).addCommand(
        new ShootAuto()
      ).build();
  }

  private static AutonomousRoutine getFourBallAuto() {
    return new AutoRoutineBuilder(2.5, 5)
      .addTrajectoryCommand(
        new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)),
        new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(270)),
        true
      ).addCommand(
        new RotateToAngle(90)
      ).addCommand(
        new ShootAuto()
      ).addCommand(
        new RotateToAngle(140, 15)
      ).addTrajectoryCommand(
        new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(140)),
        new Pose2d(5.5, 1.7, Rotation2d.fromDegrees(140))
      ).addTrajectoryCommand(
        new Pose2d(5.5, 1.7, Rotation2d.fromDegrees(140)),
        new Pose2d(1.75, 1.3, Rotation2d.fromDegrees(170))
      ).addCommand(
        new RotateToAngle(110, 15)
      ).addTrajectoryCommand(
        new Pose2d(1.75, 1.3, Rotation2d.fromDegrees(110)),
        new Pose2d(6.22, 1.8, Rotation2d.fromDegrees(50))
      ).addCommand(
        new ShootAuto()
      ).build();
  }

  private static AutonomousRoutine getFiveBallAuto() {
    return new AutoRoutineBuilder(3.5, 7)
      .setStartingPose(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(90))
      ).addCommand(
        new ShootAuto()
      ).addCommand(
        new RotateToAngle(270, 20)
      ).addTrajectoryCommand(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(270)),
        new Pose2d(7.65, 0.4, Rotation2d.fromDegrees(270))
      ).addCommand(
        new RotateToAngle(140, 15)
      ).addTrajectoryCommand(
        new Pose2d(7.65, 0.4, Rotation2d.fromDegrees(140)),
        new Pose2d(5.5, 1.7, Rotation2d.fromDegrees(140))
      ).addCommand(
        new ShootAuto()
      ).addCommand(
        new RotateToAngle(40, 10)
      ).addCommand(
        new RotateToAngle(-143, 10)
      ).addTrajectoryCommand(
        new Pose2d(5.5, 1.7, Rotation2d.fromDegrees(-143)),
        new Pose2d(1.75, 1.3, Rotation2d.fromDegrees(-143))
      ).addCommand(
        new RotateToAngle(-143)
      ).addCommand(
        new WaitCommand(0.5) // WAIT FOR HUMAN PLAYER TO FEED IN
      ).addTrajectoryCommand(
        new Pose2d(1.75, 1.3, Rotation2d.fromDegrees(-143)),
        new Pose2d(6.22, 1.8, Rotation2d.fromDegrees(50))
      ).addCommand(
        new ShootAuto()
      ).build();
  }
}
