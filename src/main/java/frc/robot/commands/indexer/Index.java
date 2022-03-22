// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Index extends CommandBase {
  /** Creates a new Index. */
  public Index() {
    addRequirements(RobotContainer.indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (!RobotContainer.switchbox.manualIndex()) {
    // int numCargo = RobotContainer.pixy.getNumCargo();
    //   if (!RobotContainer.indexer.feederBeamBreak.get()) {
    //     RobotContainer.indexer.index();
    //   } else {
    //     if (numCargo == 2) {
    //       if (RobotContainer.pixy.nextCargoLeft()) {
    //         RobotContainer.indexer.setLeft(0.9);
    //         RobotContainer.indexer.setRight(-0.7);
    //       } else {
    //         RobotContainer.indexer.setLeft(-0.7);
    //         RobotContainer.indexer.setRight(0.9);
    //       }
    //     } else if (numCargo == 1) {
    //       if (RobotContainer.pixy.nextCargoLeft()) {
    //         RobotContainer.indexer.setLeft(0.9);
    //         RobotContainer.indexer.setRight(-0.7);
    //       } else {
    //         RobotContainer.indexer.setLeft(-0.7);
    //         RobotContainer.indexer.setRight(0.9);
    //       }
    //     } else if (numCargo == 0) {
    //       RobotContainer.indexer.index();
    //     }
    //   }
    // } else {
    RobotContainer.indexer.index();
    //}

    if (RobotContainer.pixy.hasBlueInFrame() || RobotContainer.pixy.hasRedInFrame()) {
      RobotContainer.shooter.hasShotAuto = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
