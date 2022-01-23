/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final AHRS navx = new AHRS();

  public static final SwerveDrive swerveDrive = new SwerveDrive();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  //public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());

    var tab = Shuffleboard.getTab("Match");
    tab.addNumber("X", xboxController::getLeftX);
    tab.addNumber("Y", xboxController::getLeftY);
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
    Button autoCalibrateTeleop = new Button(
        () -> (!CenterSwerveModules.hasCalibrated() && RobotState.isTeleop() && RobotState.isEnabled()));
    autoCalibrateTeleop.whenPressed(new CenterSwerveModules());

    JoystickButton forceCalibrate = new JoystickButton(xboxController, XboxController.Button.kBack.value);
    forceCalibrate.whenPressed(new CenterSwerveModules());

    JoystickButton rotateToTarget = new JoystickButton(xboxController, XboxController.Button.kY.value);
    rotateToTarget.whenHeld(new RotateToTargetWhileDriving());

    Button holdAngle = new Button(() -> xboxController.getAButton());
    holdAngle.whenHeld(new HoldAngleWhileDriving());

    Button holdAngleWithJoystick = new Button(() -> ( (Math.abs(xboxController.getRightX()) < 0.05) || xboxController.getAButton()));
    holdAngleWithJoystick.whenHeld(new HoldAngleWhileDriving());

    Button resetNavxButton = new Button(xboxController::getStartButton);
    resetNavxButton.whenPressed( () -> swerveDrive.resetNavx(), swerveDrive);

    Button slowDrive = new Button( () -> (xboxController.getLeftTriggerAxis() > 0.3) );
    slowDrive.whenPressed( () -> {SwerveDrive.kMaxAngularSpeed = Math.PI / 2; SwerveDrive.kMaxSpeed = 2; } );
    slowDrive.whenReleased( () -> {SwerveDrive.kMaxAngularSpeed = Math.PI; SwerveDrive.kMaxSpeed = 3.5; } );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int id) {
    Command autonCommand;

    switch (id) {
      default:
        autonCommand = null;
      break;
      case 1:
        autonCommand = AutoCommands.twoBall();
      break;
      case 2:
        autonCommand = AutoCommands.threeBallRace();
      break;
      case 3:
        autonCommand = AutoCommands.threeBall();
      break;
    }
    return autonCommand;
  }

  public Pose2d getStartingPose(int id) {
    Pose2d startPose;
    switch (id) {
      default:
        startPose = new Pose2d();
      break;
      case 1:
        startPose = new Pose2d(7.65, 2, Rotation2d.fromDegrees(270));
      break;
      case 2:
        startPose = new Pose2d(7.4, 2.0, Rotation2d.fromDegrees(55));
      break;
      case 3:
        startPose = new Pose2d(7.4, 2.0, Rotation2d.fromDegrees(55));
      break;

    }
    return startPose;
  }
}