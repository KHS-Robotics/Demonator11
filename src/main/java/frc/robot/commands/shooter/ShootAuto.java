// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class ShootAuto extends Shoot {
  Timer timer = new Timer();

  public ShootAuto() {
    super();
    
    addRequirements(RobotContainer.intake);
  }

  @Override
  public void execute() {
    super.execute();

    // if(RobotContainer.pixy.hasBlueInFrame() || RobotContainer.pixy.hasRedInFrame()) {
    //   timer.reset();
    // }
  }

  @Override
  public void initialize() {
    super.initialize();
    timer.start();

    RobotContainer.intake.stop();
    RobotContainer.intake.setPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setHood(0.75);
    RobotContainer.indexer.stopFeeder();
    
    RobotContainer.intake.intake();
    RobotContainer.intake.setPosition(-14);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(RobotContainer.pixy.hasBlueInFrame() || RobotContainer.pixy.hasRedInFrame()) && timer.hasElapsed(3);
  }
}
