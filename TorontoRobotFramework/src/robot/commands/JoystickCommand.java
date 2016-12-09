
package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import robot.Robot;

/**
 *
 */
public class JoystickCommand extends Command {

	private boolean wasStartDriveStraightButtonReleased = false;
	
    public JoystickCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassisSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    
    	if (wasStartDriveStraightButtonReleased) {
	    	if (Robot.oi.getStartDriveStraightCommand()) {
	    		Scheduler.getInstance().add(new DriveStraightCommand(3, .5, 5));
	    		wasStartDriveStraightButtonReleased = false;
	    		return;
	    	}
    	}
    	else {
    		wasStartDriveStraightButtonReleased = 
    				! Robot.oi.getStartDriveStraightCommand();
    	}
    	
    	if (Robot.oi.getStartDriveStraightWithGyroCommand()) {
    		Scheduler.getInstance().add(new DriveStraightWithGyroCommand(12, .5, 10));
    	}
    	
    	if (Robot.oi.getDriverRumbleStart()) { Robot.oi.setDriverRumble(0.8); }
    	else  								 { Robot.oi.setDriverRumble(0); }
    	
    	double speed = Robot.oi.getSpeed();
    	double turn  = Robot.oi.getTurn();
    	
    	if (Math.abs(turn) > 0.05) {
    		Robot.chassisSubsystem.setMotorSpeed(speed, turn);
    	}
    	else {
        	Robot.chassisSubsystem.setBothMotorSpeeds(speed);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

	@Override
	protected void end() {}

	@Override
	protected void interrupted() {}
}
