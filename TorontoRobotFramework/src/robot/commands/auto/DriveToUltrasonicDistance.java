package robot.commands.auto;

import robot.Robot;
import robot.RobotConst.Direction;

public class DriveToUltrasonicDistance extends DriveOnHeadingCommand {

	private double setpointDistance;
	private Direction direction;
	
    public DriveToUltrasonicDistance(double heading, double speed, double setpointDistance) {
    
    	super(heading, Math.abs(speed));
    	this.setpointDistance = setpointDistance;
    	direction = Direction.FORWARD;
    }

    @Override
    protected void initialize() {
    	super.initialize();
    	
    	// Where am I now?
    	double currentDistance = Robot.chassisSubsystem.ultrasonicSensor.getDistance();
    	
    	// Check whether to go forward or backward to get to the desired distance
    	if (setpointDistance <= currentDistance)
    	{
    		direction = Direction.FORWARD;
    	}
    	else{
    		direction=Direction.BACKWARDS;
    	}
    		
    	// Set the direction
    	super.setDirection(direction);
    	    
    	
    }
    
	@Override
	protected boolean isFinished() {
		
		// Stop if you are there (beware of direction)
		double currentDistance = Robot.chassisSubsystem.ultrasonicSensor.getDistance();
		
			return false;
		
		
	}		
	
	@Override
	public void end() {
		// put the stopping code here
		Robot.chassisSubsystem.setMotorSpeeds(0, 0);
	}

}


