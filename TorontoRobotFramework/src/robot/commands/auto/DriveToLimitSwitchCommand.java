
package robot.commands.auto;

import com.toronto.sensors.T_LimitSwitch;

/**
 * Drive to a limit switch using the distance as a safety stop
 */
public class DriveToLimitSwitchCommand extends DriveToEncoderDistanceCommand {

	private T_LimitSwitch limitSwitch;
	
    public DriveToLimitSwitchCommand(double heading, double speed, T_LimitSwitch limitSwitch, double maxEncoderDistanceInches) {
    	super(heading, speed, maxEncoderDistanceInches);
    	this.limitSwitch = limitSwitch;
    }

    @Override
    protected void initialize() {
    	super.initialize();
    }
    
	@Override
	protected boolean isFinished() {
		
		// If at the limit, this command is finished
		if (limitSwitch.atLimit()) { return true; }

		// Check for the max distance
		return super.isFinished();
	}

}
