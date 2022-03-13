/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class RotateToTarget extends CommandBase {
  double angle;

  /**
   * Creates a new RotateToAngle.
   */
  public RotateToTarget() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setLedMode(LightMode.eOn);

    RobotContainer.swerveDrive.resetPid();
    RobotContainer.swerveDrive.stop();

    angle = RobotContainer.swerveDrive.getYaw();
  }

  @Override
  public void execute() {
    if (Limelight.isTarget()) {
      angle = RobotContainer.swerveDrive.getYaw() - Limelight.getTx();
    }

    RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;//RobotContainer.swerveDrive.atSetpoint();
  }
}
