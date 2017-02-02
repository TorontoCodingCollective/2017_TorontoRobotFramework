
package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import robot.Robot;
import robot.commands.auto.RotateToHeadingCommand;

/**
 *
 */
public class JoystickCommand extends Command {

	enum ButtonState { PRESSED, RELEASED };
	
	ButtonState driveStraightState = ButtonState.RELEASED;
	ButtonState povState           = ButtonState.RELEASED;
	ButtonState calibrateState     = ButtonState.RELEASED;
	
    public JoystickCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.chassisSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    
    	switch (driveStraightState) {
    	case RELEASED:
	    	if (Robot.oi.getStartDriveStraightCommand()) {
	    		Scheduler.getInstance().add(new DriveStraightCommand(3, .5, 5));
	    		driveStraightState = ButtonState.PRESSED;
	    		return;
	    	}
        	break;
    	case PRESSED:
    		if (! Robot.oi.getStartDriveStraightCommand()) {
    			driveStraightState = ButtonState.RELEASED;
    		}
    		break;
    	}
    	
    	switch (povState) {
    	case RELEASED:
    		double angle = Robot.oi.getRotateToAngle();
    		if (angle >= 0) {
	    		Scheduler.getInstance().add(new RotateToHeadingCommand(angle));
	    		povState = ButtonState.PRESSED;
	    		return;
	    	}
        	break;
    	case PRESSED:
    		if (Robot.oi.getRotateToAngle() < 0) {
    			povState = ButtonState.RELEASED;
    		}
    		break;
    	}
    	
    	if (Robot.oi.getDriverRumbleStart()) { Robot.oi.setDriverRumble(0.8); }
    	else  								 { Robot.oi.setDriverRumble(0); }
    	
    	switch (calibrateState) {
    	case RELEASED:
	    	if (Robot.oi.getCalibrate()) {
	    		Robot.chassisSubsystem.calibrateGyro();
	    		calibrateState = ButtonState.PRESSED;
	    	}
	    	break;
    	case PRESSED:
    		if (! Robot.oi.getCalibrate()) {
    			calibrateState = ButtonState.RELEASED;
    		}
    	}
	    	
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
