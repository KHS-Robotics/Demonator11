// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetIntake extends CommandBase {
  IntakeState state;

  /** Creates a new SetIntake. */
  public SetIntake(IntakeState state) {
    addRequirements(RobotContainer.intake);
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    RobotContainer.intake.setIdleMode(IdleMode.kBrake);

    switch(state) {
      case kDown:
        RobotContainer.intake.setPosition(-18);
        break;
      case kUp:
        RobotContainer.intake.setPosition(0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (state) {
      case kDown:
        RobotContainer.intake.setIdleMode(IdleMode.kCoast);
        RobotContainer.intake.stopPosMotor();
        break;
      case kUp:
        RobotContainer.intake.setIdleMode(IdleMode.kBrake);
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intake.atSetpoint();
  }

  public enum IntakeState {
    kDown,
    kUp
  }
}
