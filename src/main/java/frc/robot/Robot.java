/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoRoutineBuilder.AutonomousRoutine;
import frc.robot.commands.CenterSwerveModules;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;

  Command autonCommand;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (RobotContainer.pixy != null) {
      RobotContainer.pixy.updateCargoInFrame();
    }
  }

  @Override
  public void disabledInit() {
    Limelight.setLedMode(LightMode.eOff);

    SwerveDrive.kMaxSpeed = 3.5;
    SwerveDrive.kMaxAngularSpeed = Math.PI;
  }

  @Override
  public void disabledPeriodic() {
    if (RobotContainer.xboxController.getXButtonPressed()) {
      Limelight.setLedMode(LightMode.eOn);
    } else if (RobotContainer.xboxController.getYButtonPressed()) {
      Limelight.setLedMode(LightMode.eOff);
    }
  }

  @Override
  public void autonomousInit() {
    if (autonCommand != null) {
      autonCommand.cancel();
      CommandScheduler.getInstance().run();
    }
    Limelight.setLedMode(LightMode.eOn);
    AutonomousRoutine selectedAuton = RobotContainer.getCommand((int) RobotContainer.id.getDouble(0));

    RobotContainer.swerveDrive.resetNavx(selectedAuton.getStartingPose());

    autonCommand = (new CenterSwerveModules(false).andThen(new InstantCommand(() -> RobotContainer.intake.intake())).andThen(selectedAuton.getAsCommand()));
    autonCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    Limelight.setLedMode(LightMode.eOff);
    RobotContainer.swerveDrive.setOffset(0);

    if (autonCommand != null) {
      autonCommand.cancel();
    }
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
