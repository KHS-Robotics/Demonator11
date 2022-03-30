package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.rotate.LeadShotsWhileDriving;
import frc.robot.vision.Limelight;

public class ShootMoving extends Shoot{


    public ShootMoving() {
        super();

    }

    @Override
    public void execute() {
        speed = LeadShotsWhileDriving.getSpeed();
        angle = LeadShotsWhileDriving.getHoodAngle();

        double vX = Math.cos(angle) * speed;
        double initDrag = 0.2 * 1.225 * 0.0145564225 * Math.PI * vX * vX / 0.27;
        double time = LeadShotsWhileDriving.getDistNew() / (speed * Math.cos(angle));

        RobotContainer.shooter.setHoodAngle((Math.PI / 2) - LeadShotsWhileDriving.getHoodAngle());
        RobotContainer.shooter.setShooter(msToRPM(speed + (initDrag * time * time * 0.5)));
        RobotContainer.indexer.index();

        if ( RobotContainer.shooter.atSetpoint(tolerance)) {//RobotContainer.shooter.getVelocity() > msToRPM(minError) && RobotContainer.shooter.getVelocity() < msToRPM(maxError)) {
            if(debounce.hasElapsed(0.1)) {
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
}
