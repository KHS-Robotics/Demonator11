/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoRoutineBuilder.AutonomousRoutine;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.commands.intake.SetIntake;
import frc.robot.commands.intake.SetIntake.IntakeState;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;
  public static Alliance color;

  Command autonCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer(); 
    color = DriverStation.getAlliance();
    Limelight.setLedMode(LightMode.eOn);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    SwerveDrive.kMaxSpeed = 3.5;
    SwerveDrive.kMaxAngularSpeed = 1.5 * Math.PI;
  }

  @Override
  public void disabledPeriodic() {
    color = DriverStation.getAlliance();

    if(RobotContainer.joystick.disableLimelight()) {
      Limelight.setLedMode(LightMode.eOff);
    } else if (RobotContainer.joystick.enableLimelight()) {
      Limelight.setLedMode(LightMode.eOn);
    }
    
    if (RobotContainer.switchbox.climb() && RobotContainer.switchbox.shoot()) {
      RobotContainer.climber.resetPos();
      RobotContainer.intake.resetPos();
    }
  }

  @Override
  public void autonomousInit() {
    RobotContainer.intake.resetPos();

    Limelight.setLedMode(LightMode.eOn);
    if (autonCommand != null) {
      autonCommand.cancel();
      CommandScheduler.getInstance().run();
    }
    AutonomousRoutine selectedAuton = RobotContainer.getCommand((int) RobotContainer.id.getDouble(0));

    RobotContainer.swerveDrive.resetNavx(selectedAuton.getStartingPose());

    autonCommand = new CenterSwerveModules(false)
    .andThen(new InstantCommand(() -> RobotContainer.shooter.setHood(0.75))
    .alongWith(new SetIntake(IntakeState.kDown))
    .alongWith(new InstantCommand(() -> RobotContainer.intake.intake()))
    .alongWith(
      new InstantCommand( () -> RobotContainer.indexer.setFeeder(-0.9))
      .andThen(new WaitCommand(0.5))
      .andThen(new InstantCommand( () -> RobotContainer.indexer.stopFeeder()))
    )
    .alongWith(selectedAuton.getAsCommand()) );
    
    autonCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    if (autonCommand != null) {
      autonCommand.cancel();
    }

    RobotContainer.intake.stop();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }
}
