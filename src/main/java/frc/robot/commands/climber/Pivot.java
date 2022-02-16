package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Pivot extends CommandBase {
  Angle angle;

  public Pivot(Angle angle) {
    this.angle = angle;
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  @Override
  public void initialize() {
    double setpoint = 0;
    switch (angle) {
      case Rest:
        setpoint = 0;
        break;
      case Tilt:
        setpoint = 0;
        break;
      case Straight:
        setpoint = 0;
        break;
    }
    
    RobotContainer.climber.pivot(setpoint);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return RobotContainer.climber.pivotAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {}
}
