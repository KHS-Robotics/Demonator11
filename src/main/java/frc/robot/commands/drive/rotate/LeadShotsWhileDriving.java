/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.rotate;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShootMoving;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LightMode;

public class LeadShotsWhileDriving extends CommandBase {
    private double angle, offsetLimelightAngle, dist;
    private boolean isFieldOriented;
    private static double distNew, angleNew, hoodAngle, speed;
    private static double targetHeight = Constants.TARGET_HEIGHT;
    private static double robotHeight = Constants.ROBOT_HEIGHT;
    private static double limelightHeight = Constants.LIMELIGHT_HEIGHT;
    private static double limelightAngle = Constants.LIMELIGHT_ANGLE;
    int tolerance;

    /**
     * Creates a new RotateToAngle.
     */
    public LeadShotsWhileDriving() {
        addRequirements(RobotContainer.swerveDrive);

        var tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Angle New", () -> angleNew);

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
        if(Limelight.isTarget()) {

            dist = (targetHeight - limelightHeight) / (Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle)) * Math.cos(Math.toRadians((Limelight.getTx())))) + 0.91;
        }
        distNew = dist;
        refineShot(dist, 7);
        //distNew -= Math.abs(RobotContainer.swerveDrive.getChassisSpeeds().vxMetersPerSecond/3);
        angleNew -= (Math.PI / 20) * RobotContainer.swerveDrive.getChassisSpeeds().vxMetersPerSecond;
        offsetLimelightAngle = Math.toDegrees(angleNew) + Limelight.getTx();
        if (Limelight.isTarget()) { // && Math.abs(angle - (RobotContainer.swerveDrive.getYaw() - offsetLimelightAngle)) > 1) {
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

    public static void refineShot(double dist, int iterations) {
        for(int i = 0; i < iterations; i++) {
            if (distNew > 2.7) {
                hoodAngle = Math.atan(((Math.tan(-0.698131701) * (distNew)) - (2 * (targetHeight - robotHeight))) / -distNew);
            } else {
                hoodAngle = Math.atan(((Math.tan(-1.21) * (distNew)) - (2 * (targetHeight - robotHeight))) / -distNew);
            }

            double result = (targetHeight - robotHeight);
            speed = ShootMoving.ridders(3.5, 13.5, hoodAngle, distNew, result, 15);

            //estimated travel time based on last iteration
            double t = distNew /  (speed * Math.cos(hoodAngle));
            //angle of the velocity of the bot/ball
            double a1 = Math.atan2(RobotContainer.swerveDrive.getChassisSpeeds().vyMetersPerSecond, RobotContainer.swerveDrive.getChassisSpeeds().vxMetersPerSecond) - (Math.toRadians(Limelight.getTx()) + Math.PI / 2);
            //distance travelled by ball from velocity of bot when shot
            double dist2 = Math.sqrt(Math.pow(RobotContainer.swerveDrive.getChassisSpeeds().vyMetersPerSecond, 2) + Math.pow(RobotContainer.swerveDrive.getChassisSpeeds().vxMetersPerSecond, 2)) * t;
            //finds distance for robot to aim at
            distNew = Math.sqrt(Math.pow(dist2, 2) + Math.pow(dist, 2) - 2 * dist2 * dist * Math.cos(a1));
            //finds angle for robot to aim at
            angleNew = Math.asin(Math.sin(a1) * dist2 / distNew);
        }
    }

    public static double getSpeed() { return speed; }
    public static double getDistNew() { return distNew; }
    public static double getHoodAngle() { return hoodAngle; }


}
