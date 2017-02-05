
package robot.commands.auto;

import robot.Robot;

/**
 *
 */
public class DriveToEncoderDistanceCommand extends DriveOnHeadingCommand {

	private double encoderDistanceInches;
	
    public DriveToEncoderDistanceCommand(double heading, double speed, double encoderDistanceInches) {
    	super(heading, speed);
    	this.encoderDistanceInches = encoderDistanceInches - 2.0;  // allow 2 inches overshoot for stopping.
    }

    @Override
    protected void initialize() {
    	super.initialize();
    	Robot.chassisSubsystem.resetEncoders();
    }
    
	@Override
	protected boolean isFinished() {
		if (this.setSpeed >= 0) {
			if (Robot.chassisSubsystem.getEncoderDistanceInches() > this.encoderDistanceInches) {
				return true;
			}
		} else {
			if (Robot.chassisSubsystem.getEncoderDistanceInches() < this.encoderDistanceInches) {
				return true;
			}
		}
		return false;
	}

}
