/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class RotateToTargetWhileDriving extends CommandBase {
  private double angle, defaultAngle = 0;
  private boolean isFieldOriented;

  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTargetWhileDriving() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerveDrive.resetPid();
    Limelight.setLedMode(LightMode.eOn);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.navx.getDisplacementX() + RobotContainer.swerveDrive.startingPose.getX() > 8.2) {
    //   defaultAngle = 180;
    // } else {
    //   defaultAngle = 0;
    // }
    defaultAngle = 0;
    
    if (Limelight.isTarget()) {
      angle = RobotContainer.swerveDrive.getYaw() - Limelight.getTx();
    } else {
      angle = defaultAngle;
    }

    var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getLeftY()) * SwerveDrive.kMaxSpeed;

    var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getLeftX()) * SwerveDrive.kMaxSpeed;

    isFieldOriented = (!RobotContainer.xboxController.getLeftBumper());

    RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, isFieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Limelight.setLedMode(LightMode.eOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
