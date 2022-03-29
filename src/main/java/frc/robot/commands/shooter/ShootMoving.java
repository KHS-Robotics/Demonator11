package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.rotate.LeadShotsWhileDriving;
import frc.robot.vision.Limelight;

public class ShootMoving extends Shoot{
    private static double distNew, angleNew, angle, speed;
    private static double targetHeight = Constants.TARGET_HEIGHT;
    private static double robotHeight = Constants.ROBOT_HEIGHT;
    private static double limelightHeight = Constants.LIMELIGHT_HEIGHT;
    private static double limelightAngle = Constants.LIMELIGHT_ANGLE;
    int tolerance;

    public ShootMoving() {
        super();

    }

    @Override
    public void execute() {
        if(Limelight.isTarget()) {
            dist = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(Limelight.getTy() + limelightAngle)) + 0.91 + 0.15;
        }

        refineShot(dist, 10);


        double vX = Math.cos(angle) * speed;
        double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
        double time = dist / (speed * Math.cos(angle));

        RobotContainer.shooter.setHoodAngle((Math.PI / 2) - angle);
        RobotContainer.shooter.setShooter(msToRPM(speed + (initDrag * time * time * 0.5)));
        RobotContainer.indexer.index();

        if ( RobotContainer.shooter.atSetpoint(tolerance)) {//RobotContainer.shooter.getVelocity() > msToRPM(minError) && RobotContainer.shooter.getVelocity() < msToRPM(maxError)) {
            if(debounce.hasElapsed(0.2)) {
                RobotContainer.indexer.feed();
            }
        } else {
            debounce.reset();
            RobotContainer.indexer.stopFeeder();
        }



    }

    @Override
    public void initialize() {
        super.initialize();
        distNew = dist;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
        RobotContainer.shooter.setHood(0.5);
        RobotContainer.indexer.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public static void refineShot(double dist, int iterations) {
        for(int i = 0; i < iterations; i++) {
            if (distNew > 2.7) {
                angle = Math.atan(((Math.tan(-0.698131701) * (distNew)) - (2 * (targetHeight - robotHeight))) / -distNew);
            } else {
                angle = Math.atan(((Math.tan(-1.21) * (distNew)) - (2 * (targetHeight - robotHeight))) / -distNew);
            }

            double result = (targetHeight - robotHeight);
            speed = ridders(3.5, 13.5, angle, distNew, result, 20);

            //estimated travel time based on last iteration
            double t = distNew * speed * Math.cos(angle);
            //angle of the velocity of the bot/ball
            double a1 = (Math.PI / 2) - Math.atan(RobotContainer.navx.getVelocityX() / RobotContainer.navx.getVelocityY()) + Math.toRadians(Limelight.getTx());
            //distance travelled by ball from velocity of bot when shot
            double dist2 = Math.sqrt(Math.pow(RobotContainer.navx.getVelocityX(), 2) + Math.pow(RobotContainer.navx.getVelocityY(), 2)) * t;
            //finds distance for robot to aim at
            distNew = Math.sqrt(Math.pow(dist2, 2) + Math.pow(dist, 2) - 2 * dist2 * dist * Math.cos(a1));
            //finds angle for robot to aim at
            angleNew = Math.asin(Math.sin(a1) * dist2 / distNew);
        }
    }

    public static double getAngleNew() {
        return angleNew;
    }
}
