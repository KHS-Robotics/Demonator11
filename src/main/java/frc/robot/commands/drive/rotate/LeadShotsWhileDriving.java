/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShootMoving;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class LeadShotsWhileDriving extends CommandBase {
    private double angle, offsetLimelightAngle;
    private boolean isFieldOriented;

    /**
     * Creates a new RotateToAngle.
     */
    public LeadShotsWhileDriving() {
        addRequirements(RobotContainer.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.swerveDrive.resetPid();
        Limelight.setLedMode(LightMode.eOn);

        angle = RobotContainer.swerveDrive.getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        offsetLimelightAngle = Limelight.getTx() + ShootMoving.getAngleNew();
        if (Limelight.isTarget() && Math.abs(angle - (RobotContainer.swerveDrive.getYaw() - offsetLimelightAngle)) > 2) {
            angle = RobotContainer.swerveDrive.getYaw() - offsetLimelightAngle;
        }

        var xSpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getLeftY()) * SwerveDrive.kMaxSpeed;

        var ySpeed = RobotContainer.swerveDrive.sensControl(-RobotContainer.xboxController.getLeftX()) * SwerveDrive.kMaxSpeed;

        isFieldOriented = (!RobotContainer.xboxController.getLeftBumper());

        RobotContainer.swerveDrive.holdAngleWhileDriving(xSpeed, ySpeed, angle, isFieldOriented);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }


}
