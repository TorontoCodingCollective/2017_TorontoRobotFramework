
package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotConst;
import robot.subsystems.ChassisSubsystem;

/**
 *
 */
public class TurnToAngle extends Command {

	private double setSpeed;
	private double timeout;
	private double targetAngle = -1;
	private double angleChange;
	
    public TurnToAngle(double turnAngle, double speed, double timeout) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassisSubsystem);
        this.setSpeed        = speed;
        this.timeout         = timeout;
        this.angleChange 	 = turnAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	targetAngle = Robot.chassisSubsystem.getAngle() + angleChange;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	double angleError = targetAngle - Robot.chassisSubsystem.getAngle();
    	
    	if (angleError > 180.0)  { angleError -= 360.0; }
    	if (angleError < -180.0) { angleError += 360.0; }
    	
		double leftSpeed  = setSpeed;
		double rightSpeed = setSpeed;

		// Slow down one motor based on the error.
		if (angleError > 0) {
    		rightSpeed -= angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (rightSpeed < -setSpeed) {
    			 rightSpeed = -setSpeed;
    			 leftSpeed = setSpeed;
    		}
    	}
    	else {
    		leftSpeed -=  -angleError * RobotConst.GYRO_PROPORTIONAL_GAIN * setSpeed;
    		if (leftSpeed < -setSpeed) {
    			leftSpeed = -setSpeed;
    			rightSpeed = setSpeed;
    		}
    	}
    	
    	Robot.chassisSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	// Check for a timeout before the distance
    	if (timeSinceInitialized() > timeout) { return true; }
    	
    	// Look for Joystick movement - and then end
    	if (Robot.oi.isDriverAction()) { return true; }
    	
    	return targetAngle == Robot.chassisSubsystem.getAngle();
    }

	@Override
	protected void end() {
		Robot.chassisSubsystem.setBothMotorSpeeds(0);
	}

	@Override
	protected void interrupted() {}
}
