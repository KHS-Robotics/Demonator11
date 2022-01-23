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
import frc.robot.vision.Limelight.LightMode;
import frc.robot.vision.pixy.PixyCam;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;


public class Robot extends TimedRobot {
  RobotContainer robotContainer;

  Command autonCommand;
  private static PixyCam pixy;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    pixy = new PixyCam();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    Limelight.setLedMode(LightMode.eOff);

    SwerveDrive.kMaxSpeed = 3.5;
    SwerveDrive.kMaxAngularSpeed = Math.PI;
    pixy.setLamp(false);
  }

  @Override
  public void disabledPeriodic() { 
    if(RobotContainer.xboxController.getXButtonPressed()) {
      Limelight.setLedMode(LightMode.eOn);
    } else if(RobotContainer.xboxController.getYButtonPressed()) {
      Limelight.setLedMode(LightMode.eOff);
    }
  }

  @Override
  public void autonomousInit() {
    Limelight.setLedMode(LightMode.eOn);
    pixy.setLamp(true);
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void teleopInit() {
    Limelight.setLedMode(LightMode.eOff);
    RobotContainer.swerveDrive.setOffset(0);
    pixy.setLamp(true);

    if(autonCommand != null) {
      autonCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    RobotContainer.swerveDrive.updateOdometry();
  }

  @Override
  public void testInit() {
    
  }

  @Override
  public void testPeriodic() {
    
  }
}