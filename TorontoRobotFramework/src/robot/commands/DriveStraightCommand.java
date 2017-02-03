
package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.RobotConst;

/**
 *
 */
public class DriveStraightCommand extends Command {

	private double encoderDistance;
	private double speed;
	private double timeout;
	
    public DriveStraightCommand(double distance, double speed, double timeout) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassisSubsystem);
        this.encoderDistance = Math.abs(distance) 
        		* RobotConst.DRIVE_ENCODER_COUNTS_PER_FT - 200;
        this.speed           = speed;
        this.timeout         = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.chassisSubsystem.resetEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	Robot.chassisSubsystem.setBothMotorSpeeds(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	// Check for a timeout before the distance
    	if (timeSinceInitialized() > timeout) { return true; }
    	
    	// Look for Joystick movement - and then end
    	if (Robot.oi.isDriverAction()) { return true; }
    	
    	return Math.abs(Robot.chassisSubsystem.getEncoderDistanceInches()) >= encoderDistance;
    }

	@Override
	protected void end() {
//		Robot.chassisSubsystem.setBothMotorSpeeds(0);
	}

	@Override
	protected void interrupted() {}
}
