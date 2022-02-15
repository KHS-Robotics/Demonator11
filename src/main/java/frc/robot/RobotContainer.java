/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoRoutineBuilder.AutonomousRoutine;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToAngle;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.pixy.PixyCam;

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
  public static final AHRS navx = new AHRS();

  public static final PixyCam pixy = new PixyCam();

  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Indexer indexer = new Indexer();

  public static final PowerDistribution pdp = new PowerDistribution();

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
    Button autoCalibrateTeleop = new Button( () -> ( !swerveDrive.isCalibrated && RobotState.isTeleop() && RobotState.isEnabled() ) );
    autoCalibrateTeleop.whenPressed(new CenterSwerveModules(true));

    JoystickButton forceCalibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    forceCalibrate.whenPressed(new CenterSwerveModules(true));

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    Button holdAngle = new Button( xboxController::getAButton );
    holdAngle.whenHeld(new HoldAngleWhileDriving());

    Button resetNavxButton = new Button(xboxController::getStartButton);
    resetNavxButton.whenPressed( () -> swerveDrive.resetNavx(), swerveDrive);

    Button slowDrive = new Button( () -> (xboxController.getLeftTriggerAxis() > 0.3) );
    slowDrive.whenPressed( () -> {SwerveDrive.kMaxAngularSpeed = Math.PI / 2; SwerveDrive.kMaxSpeed = 2; } );
    slowDrive.whenReleased( () -> {SwerveDrive.kMaxAngularSpeed = Math.PI; SwerveDrive.kMaxSpeed = 3.5; } );

    Button intakeBall = new Button( switchbox::intake );
    intakeBall.whileHeld( () -> intake.intake(), intake );
    intakeBall.whenReleased( () -> intake.stop(), intake );

    Button outtakeBall = new Button( switchbox::outtake );
    outtakeBall.whileHeld( () -> intake.reverse(), intake );
    outtakeBall.whenReleased( () -> intake.stop(), intake );

    Button shoot = new Button(switchbox::shoot);
    shoot.whenHeld(new ManualShoot(1500));
  }

  public static AutonomousRoutine getCommand(int id) {

    switch(id) {
      default:
        return new AutonomousRoutine(new Pose2d(), new SequentialCommandGroup());
      case 0:
        return getTwoBallAuto();
      case 1:
        return getThreeBallClose();
      case 2:
        return getThreeBallFar();
      case 4:
        return getFourBallAuto();
    }
  }

  private static AutonomousRoutine getTwoBallAuto() {
    return  new AutoRoutineBuilder()
    .addTrajectoryCommand(
      new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)), 
      new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(270)),
      true
    ).addCommand(new RotateToAngle(90)
    //SHOOT
    ).build();
  }

  private static AutonomousRoutine getThreeBallClose() {
    return new AutoRoutineBuilder()
    //SHOOT
    .addTrajectoryCommand(
      new Pose2d(7.85, 1.80, Rotation2d.fromDegrees(90)), 
      new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(180)), 
      true
    ).addCommand(
      new RotateToAngle(318.27)
    ).addTrajectoryCommand(
      new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(318.27)),
      new Pose2d(7.24, 0.67, Rotation2d.fromDegrees(318.27))
    ).addCommand(
      new RotateToAngle(90)
    ) //SHOOT
    .build();
  }

  private static AutonomousRoutine getThreeBallFar() {
    return new AutoRoutineBuilder()
    //SHOOT
    .addTrajectoryCommand(
      new Pose2d(7.85, 1.80, Rotation2d.fromDegrees(90)), 
      new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(180)), 
      true
    ).addTrajectoryCommand(
      new Pose2d(5.54, 1.90, Rotation2d.fromDegrees(180)), 
      new Pose2d(1.75, 1.30, Rotation2d.fromDegrees(180))
    ).addTrajectoryCommand(
      new Pose2d(1.75, 1.30, Rotation2d.fromDegrees(180)), 
      new Pose2d(6.7, 1.4, Rotation2d.fromDegrees(45))
    ) //SHOOT
    .build();
  }

  private static AutonomousRoutine getFourBallAuto() {
    return new AutoRoutineBuilder()
    .addTrajectoryCommand(
      new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)), 
      new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(270)),
      true
    ).addCommand(
      new RotateToAngle(90)
    ) //SHOOT
    .addTrajectoryCommand(
      new Pose2d(7.65, 0.6, Rotation2d.fromDegrees(90)),
      new Pose2d(5.4, 1.7, Rotation2d.fromDegrees(140))
    ).addTrajectoryCommand(
      new Pose2d(5.4, 1.7, Rotation2d.fromDegrees(140)),
      new Pose2d(1.7, 1.3, Rotation2d.fromDegrees(180))
    ).addTrajectoryCommand(
      new Pose2d(1.7, 1.3, Rotation2d.fromDegrees(180)), 
      new Pose2d(6.22, 1.8, Rotation2d.fromDegrees(50))
    ) //SHOOT
    .build();
  }
}