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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.AutoRoutineBuilder.AutonomousRoutine;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.climber.*;
import frc.robot.commands.climber.Elevate.Level;
import frc.robot.commands.climber.Pivot.Angle;
import frc.robot.commands.drive.DriveSwerveWithXbox;
import frc.robot.commands.drive.rotate.HoldAngleWhileDriving;
import frc.robot.commands.drive.rotate.LeadShotsWhileDriving;
import frc.robot.commands.drive.rotate.RotateToAngle;
import frc.robot.commands.drive.rotate.RotateToTarget;
import frc.robot.commands.drive.rotate.RotateToTargetWhileDriving;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.intake.SetIntake.IntakeState;
import frc.robot.commands.shooter.AutoAdjustHood;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAuto;
import frc.robot.commands.shooter.ShootMoving;
import frc.robot.commands.indexer.Index;
import frc.robot.subsystems.*;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

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

  public static final PowerDistribution pdp = new PowerDistribution();

  public static final PixyCam pixy = new PixyCam();

  public static final SwerveDrive swerveDrive = new SwerveDrive();
  public static final Intake intake = new Intake();
  public static final Shooter shooter = new Shooter();
  public static final Indexer indexer = new Indexer();
  public static final Climber climber = new Climber();

  public static final XboxController xboxController = new XboxController(RobotMap.XBOX_PORT);
  public static final SwitchBox switchbox = new SwitchBox(RobotMap.SWITCHBOX_PORT);
  public static final OperatorStick joystick = new OperatorStick(RobotMap.JOYSTICK_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive.setDefaultCommand(new DriveSwerveWithXbox());
    shooter.setDefaultCommand(new RampShooter());
    indexer.setDefaultCommand(new Index());
    
    var tab = Shuffleboard.getTab("Match");
    id = tab.add("AUTO ID", 0).getEntry();
    tab.addNumber("dist", () -> (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(Limelight.getTy() + Constants.LIMELIGHT_ANGLE)) + 0.91);
    tab.addBoolean("Limelight See's Target", Limelight::isTarget);

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

    Button slowDrive = new Button(() -> (xboxController.getLeftTriggerAxis() > 0.3 || xboxController.getRightTriggerAxis() > 0.3));
    slowDrive.whenPressed(() -> {
      SwerveDrive.kMaxAngularSpeed = Math.PI;
      SwerveDrive.kMaxSpeed = 2;
    });
    slowDrive.whenReleased(() -> {
      SwerveDrive.kMaxAngularSpeed = 2 * Math.PI;
      SwerveDrive.kMaxSpeed = 3.5;
    });

    Button intakeBall = new Button(() -> (switchbox.intake()  && !switchbox.outtake() ));
    intakeBall.whileHeld(() -> intake.intake());
    intakeBall.whenReleased(() -> intake.stop(), intake);

    Button outtakeBall = new Button(switchbox::outtake);
    outtakeBall.whenPressed(() -> {intake.reverse(); indexer.setFeeder(-0.8); indexer.setLeft(-0.8); indexer.setRight(-0.8); }, intake, indexer);
    outtakeBall.whenReleased(() -> {intake.stop(); indexer.stop(); }, intake, indexer);

    Button shoot = new Button(() -> ( switchbox.shoot() || xboxController.getYButton() ) );
    shoot.whenHeld(new WaitCommand(0.3).andThen(new Shoot()) );

    Button shootMoving = new Button(xboxController::getXButton);
    shootMoving.whileHeld(new LeadShotsWhileDriving().alongWith(new ShootMoving()));
    shootMoving.whenPressed(() -> {
      SwerveDrive.kMaxSpeed = 1.5;
    });
    shootMoving.whenReleased(() -> {
      SwerveDrive.kMaxSpeed = 3.5;
    });

    Button dropIntake = new Button(switchbox::intakeDown);
    dropIntake.whenHeld( new SetIntake(IntakeState.kDown));

    Button raiseIntake = new Button( () -> !switchbox.intakeDown());
    raiseIntake.whenHeld( new SetIntake(IntakeState.kUp) );

    Button raiseRPM = new Button( joystick::incrementRPM );
    raiseRPM.whenPressed(() -> shooter.incrementMultiplier() );

    Button lowerRPM = new Button( joystick::decrementRPM );
    lowerRPM.whenPressed(() -> shooter.decrementMultiplier() );

    Button resetRPM = new Button( joystick::resetRPM );
    resetRPM.whenPressed(new InstantCommand(() -> shooter.resetMultipler()));

    Button enableLimelight = new Button( joystick::disableLimelight );
    enableLimelight.whenPressed( () -> Limelight.setLedMode(LightMode.eOff) );

    Button disableLimelight = new Button( joystick::enableLimelight );
    disableLimelight.whenPressed( () -> Limelight.setLedMode(LightMode.eOn));

    Button goToZero = new Button(xboxController::getBButton);
    goToZero.whileHeld(new RotateToAngle(180));

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

    // Button indexerOff = new Button(() -> switchbox.getRawAxis(0) > 0.1);
    // indexerOff.whileHeld(new InstantCommand(() -> {indexer.stop();}, indexer));

    Button manualClimb = new Button( switchbox::climbOverride );
    manualClimb.whenHeld( new ManualClimb( () -> joystick.getElevatorSpeed(), () -> joystick.getPivotSpeed() ) );

    Button setupClimb = new Button(switchbox::climb);
    setupClimb.whenPressed( new InstantCommand(() -> climber.resetPitch() ).andThen (new UnhookElevator()).andThen(new Elevate(Level.Reach)));

    Button climbButton = new Button( joystick::climb );
    climbButton.whenPressed( new SetPivotVoltage(-0.15, climber.pivotTime).andThen(new InstantCommand(() -> climber.pivotTime = 2)).andThen(new Elevate(Level.Zero)).andThen(new InstantCommand( () -> climber.setElevatorSpeed(0) )));
    
    Button transfer = new Button( () -> joystick.getRawButton(9) );
    transfer.whenPressed( new InstantCommand( () -> climber.setElevatorSpeed(0.2) ).andThen(new WaitCommand(0.25)).andThen(new Elevate(Level.MidHeight)).andThen(new Pivot(Angle.Tilt)).andThen(new Elevate(Level.Reach)).andThen(new WaitCommand(0.5)).andThen(new Pivot(Angle.Handoff)).andThen(new SetPivotVoltage(-0.45, 2.5)) );

    Button handoff = new Button( joystick::handoff );
    handoff.whenPressed( new Elevate(Level.MidHeight));

    Button rampShooter = new Button(switchbox::rampShooter);
    rampShooter.whenPressed( new RampShooter() );
 
    Button eject = new Button(switchbox::eject);
    eject.whileHeld(() -> {shooter.setHood(0.5); shooter.setShooter(1200); indexer.setFeeder(0.8); indexer.index();}, shooter, indexer);
    eject.whenReleased(() -> {shooter.setShooter(0); indexer.setFeeder(0);});

    Button ejectWrongColor = new Button(() -> ( (!(switchbox.shoot() || xboxController.getXButton() || switchbox.stopAutoEject() || xboxController.getYButton()) && !(pixy.nextCargo().getColorAsAlliance() == Robot.color) && (pixy.nextCargo().getColorAsAlliance() != DriverStation.Alliance.Invalid))) );
    ejectWrongColor.whileHeld(() -> {if (pixy.nextCargo().getColorAsAlliance() != Robot.color) {shooter.setHood(0.5); shooter.setShooter(1000); indexer.setFeeder(0.8); indexer.index();}}, shooter, indexer );
    ejectWrongColor.whenReleased(new InstantCommand( () -> indexer.stopFeeder() ).andThen( new WaitCommand(0.2) ).andThen(new InstantCommand( () -> shooter.setShooter(0) ) ) );
  }

  public static AutonomousRoutine getCommand(int id) {

    switch (id) {
      default:
        return new AutonomousRoutine(new Pose2d(), new SequentialCommandGroup());
      case 0:
        return getTwoBallAuto();
      case 1:
        return getTwoBallUp();
      case 2:
         return getThreeBallClose();
      // case 2:
      //   return getThreeBallFar();
      // case 3:
      //   return getThreeBallMid();
      case 3:
        return getFourBallAuto();
      // case 5:
      //   return getFiveBallAuto();
      // case 6:
      //   return getAlternateFiveBall();
    }
  }

  private static AutonomousRoutine getTwoBallAuto() {
    return new AutoRoutineBuilder()
      .addTrajectoryCommand(
        new Pose2d(7.65, 2, Rotation2d.fromDegrees(270)),
        new Pose2d(7.65, 0.65, Rotation2d.fromDegrees(270)),
        true
      ).addCommand(  
        new RotateToAngle(90, 15)
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getTwoBallUp() {
    return new AutoRoutineBuilder()
    .addTrajectoryCommand(
        new Pose2d(6.1, 5.1, Rotation2d.fromDegrees(135)),
        new Pose2d(5.35, 6.00, Rotation2d.fromDegrees(135)),
        true
      ).addCommand(  
        new RotateToAngle(-45, 15)
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getThreeBallClose() {
    return new AutoRoutineBuilder()
      .setStartingPose(
        new Pose2d(7.6, 1.80, Rotation2d.fromDegrees(90))
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).addCommand( 
        new RotateToAngle(180, 15).alongWith(new InstantCommand( () -> { RobotContainer.shooter.setShooter(1582); RobotContainer.shooter.setHoodAngle(0.477675016322586); } ) )
      ).addTrajectoryCommand(
        new Pose2d(7.6, 1.80, Rotation2d.fromDegrees(180)),
        new Pose2d(5.6, 1.80, Rotation2d.fromDegrees(180))
      ).addCommand(
        new RotateToAngle(318.27, 10)
      ).addTrajectoryCommand(
        new Pose2d(5.6, 1.70, Rotation2d.fromDegrees(318.27)),
        new Pose2d(7.3, 0.3, Rotation2d.fromDegrees(318.27))
      ).addCommand(
        new RotateToAngle(90)
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getThreeBallMid() {
    return new AutoRoutineBuilder(1.5, 2)
      .addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).addCommand(
        new RotateToAngle(270, 15).alongWith(new InstantCommand( () -> { RobotContainer.shooter.setShooter(1557); RobotContainer.shooter.setHoodAngle(0.4655415803885494); } ))
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
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getThreeBallFar() {
    return new AutoRoutineBuilder()
      .setStartingPose(
        new Pose2d(6.6, 2.56, Rotation2d.fromDegrees(45))
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).addCommand(
        new RotateToAngle(-170, 10).alongWith(new InstantCommand( () -> { RobotContainer.shooter.setShooter(1612); RobotContainer.shooter.setHoodAngle(0.4909171668794452); } ))
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
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getFourBallAuto() {
    return new AutoRoutineBuilder(2.75, 5)
      .addCommand(
        new InstantCommand( () -> { RobotContainer.shooter.setHoodAngle(0.45495017309236974); } )
      ).addTrajectoryCommand(
        new Pose2d(6.64, 2.49, Rotation2d.fromDegrees(225)),
        new Pose2d(5.35, 1.95, Rotation2d.fromDegrees(197)),
        true
      ).addCommand(
        new RotateToAngle(26, 15)
        .andThen(new ShootAuto()
          .deadlineWith(new RotateToTarget())
        ).andThen(new InstantCommand( () -> { RobotContainer.shooter.setHoodAngle(0.47533473686797345); } )).andThen(new WaitCommand(0.1))
      ).addTrajectoryCommand(
        new Pose2d(5.35, 1.95, Rotation2d.fromDegrees(197)),
        new Pose2d(1.35, 0.95+0.55, Rotation2d.fromDegrees(210))
      ).addCommand(
        new WaitCommand(0.5)
      ).addTrajectoryCommand(
        new Pose2d(1.55, 0.95+0.55, Rotation2d.fromDegrees(210)),
        new Pose2d(5.61, 1.16, Rotation2d.fromDegrees(42.18))
      ).addCommand(
        new ShootAuto()
          .deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getFiveBallAuto() {
    return new AutoRoutineBuilder(3.5, 7)
      .setStartingPose(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(90))
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).addCommand(
        new RotateToAngle(270, 20).alongWith(new InstantCommand( () -> { RobotContainer.shooter.setShooter(1686); RobotContainer.shooter.setHoodAngle(0.519687052923882); } ) )
      ).addTrajectoryCommand(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(270)),
        new Pose2d(8.05, 0.6, Rotation2d.fromDegrees(270))
      ).addCommand(
        new RotateToAngle(160, 15)
      ).addTrajectoryCommand(
        new Pose2d(8.05, 0.6, Rotation2d.fromDegrees(160)),
        new Pose2d(4.5, 1.25, Rotation2d.fromDegrees(160))
      ).addCommand(
          new RotateToAngle(40, 10)
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).addCommand(
        new RotateToAngle(-140, 10).alongWith( new InstantCommand( () -> { RobotContainer.shooter.setShooter(1476); RobotContainer.shooter.setHoodAngle(0.42085501733269615); } ) )
      ).addTrajectoryCommand(
        new Pose2d(4.5, 1.25, Rotation2d.fromDegrees(-140)),
        new Pose2d(0.65, 0.15, Rotation2d.fromDegrees(-140))
      ).addCommand(
        new RotateToAngle(-160)
      ).addCommand(
        new WaitCommand(0.5) // WAIT FOR HUMAN PLAYER TO FEED IN
      ).addTrajectoryCommand(
        new Pose2d(0.65, 0.15, Rotation2d.fromDegrees(-160)),
        new Pose2d(6.22, 1.8, Rotation2d.fromDegrees(50))
      ).addCommand(
        new ShootAuto().deadlineWith(new RotateToTarget())
      ).build();
  }

  private static AutonomousRoutine getAlternateFiveBall() {
    return new AutoRoutineBuilder(3.5, 7)
      .setStartingPose(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(270))
      ).addCommand(
       new InstantCommand( () -> { RobotContainer.shooter.setShooter(1686); RobotContainer.shooter.setHoodAngle(0.519687052923882); } ) 
      ).addTrajectoryCommand(
        new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(270)),
        new Pose2d(8.05, 0.6, Rotation2d.fromDegrees(270))
      ).addCommand(
        new RotateToAngle(90, 15).andThen( new ShootAuto().deadlineWith(new RotateToTarget() ) ).andThen( new InstantCommand( () -> { RobotContainer.shooter.setShooter(1686); RobotContainer.shooter.setHoodAngle(0.519687052923882); } ) )
      ).addTrajectoryCommand(
        new Pose2d(8.05, 0.6, Rotation2d.fromDegrees(90)), 
        new Pose2d(0.65, 0.15, Rotation2d.fromDegrees(-140))
      ).addTrajectoryCommand(
        new Pose2d(0.65, 0.15, Rotation2d.fromDegrees(-140)),
        new Pose2d(4.65, 1.2, Rotation2d.fromDegrees(35))
      ).addCommand(
        new ShootAuto().deadlineWith(new AutoRoutineBuilder(0.5, 0.25)
        .createTrajectoryCommandFollowAngle(
          new Pose2d(4.65, 1.2, Rotation2d.fromDegrees(35)), 
          new Pose2d(5.12, 1.85, Rotation2d.fromDegrees(35)), 
          () -> Rotation2d.fromDegrees(swerveDrive.getYaw() - Limelight.getTx())
        ))
      ).build();
  }

  private static AutonomousRoutine getLoganFiveBall() {
    return new AutoRoutineBuilder(3.5, 7)
    .addCommand(
      new InstantCommand(() -> { shooter.setShooter(1500); })
    ).addTrajectoryCommand(
      new Pose2d(7.65, 1.8, Rotation2d.fromDegrees(270)), 
      new Pose2d(7.85, 0.6, Rotation2d.fromDegrees(270)),
      true
    ).addTrajectoryCommand(
      new Pose2d(7.85, 0.6, Rotation2d.fromDegrees(270)), 
      new Pose2d(4.65, 1.2, Rotation2d.fromDegrees(35))
    ).addCommand(
      new ShootAuto().deadlineWith(
        new AutoRoutineBuilder(0.5, 0.25)
        .createTrajectoryCommandFollowAngle(
          new Pose2d(4.65, 1.2, Rotation2d.fromDegrees(35)), 
          new Pose2d(5.12, 1.85, Rotation2d.fromDegrees(35)), 
          () -> Rotation2d.fromDegrees(swerveDrive.getYaw() - Limelight.getTx())
        )
      ).andThen(new InstantCommand(() -> { shooter.setShooter(1500); }))
    ).addTrajectoryCommand(
      new Pose2d(5.12, 1.85, Rotation2d.fromDegrees(35)), 
      new Pose2d(0.65, 0.25, Rotation2d.fromDegrees(-140))
    ).addTrajectoryCommand(
      new Pose2d(0.65, 0.25, Rotation2d.fromDegrees(-140)),
      new Pose2d(6.22, 1.8, Rotation2d.fromDegrees(50))
    ).addCommand(
      new ShootAuto().deadlineWith(new RotateToTarget())
    ).build();
  }
}