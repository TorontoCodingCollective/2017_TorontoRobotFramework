
package robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotConst;

/**
 *  The DriveOnHeading command is the base class for all of the 
 *  auto commands as it tracks a gyro heading.
 *  <p>
 *  This command should be extended for other types of auto drive commands.
 */
public class DriveOnHeadingCommand extends Command {

	private double heading;
	private double setSpeed;
	
    public DriveOnHeadingCommand(double heading, double speed) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassisSubsystem);
        this.heading  = heading;
        this.setSpeed = speed;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	
    	double angleError = heading - Robot.chassisSubsystem.getAngle();
    	
    	if (angleError > 180.0)  { angleError -= 360.0; }
    	if (angleError < -180.0) { angleError += 360.0; }
    	
		double leftSpeed  = setSpeed;
		double rightSpeed = setSpeed;

		// Slow down one motor based on the error.
		if (angleError > 0) {
    		rightSpeed -= angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (rightSpeed < -setSpeed) {
    			 rightSpeed = -setSpeed;
    		}
    	}
    	else {
    		leftSpeed -=  -angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (leftSpeed < -setSpeed) {
    			leftSpeed = -setSpeed;
    		}
    	}
    	
    	Robot.chassisSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	return false;
    }

}
