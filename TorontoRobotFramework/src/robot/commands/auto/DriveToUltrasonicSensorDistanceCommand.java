package robot.commands.auto;

import robot.Robot;

public class DriveToUltrasonicSensorDistanceCommand extends DriveOnHeadingCommand{

private double ultrasonicSensorDistance;

	
    public DriveToUltrasonicSensorDistanceCommand(double heading, double speed, double ultrasonicDistance) {
    	super(heading, speed);
    	this.ultrasonicSensorDistance= ultrasonicSensorDistance - 7.3;  // allow 2 inches overshoot for stopping.
    }

    @Override
    protected void initialize() {
    	super.initialize();
    	Robot.chassisSubsystem.ultrasonicSensor.getDistance();
    }
    
	protected boolean isFinished() {
//		if (this.setSpeed >= 0) {
//			if (Robot.chassisSubsystem.ultrasonicSeonsor.getDistance() > this.ultrasonicSensorDistance) {
//				Robot.chassisSubsystem.setMotorSpeeds(0, 0);
//				return true;
//			}
//		} else {
//			if (Robot.chassisSubsystem.getUltrasonicSensorDistance() < this.ultrasonicSensorDistance) {
//				Robot.chassisSubsystem.setMotorSpeeds(0, 0);
//				return true;
//			}
//		}
		if(Math.abs(Robot.chassisSubsystem.ultrasonicSensor.getDistance()) > Math.abs(this.ultrasonicSensorDistance)){
			Robot.chassisSubsystem.setMotorSpeeds(0, 0);
			System.out.println("Chassis subsystem UltrasonicSensor distance inches:" + Math.abs(Robot.chassisSubsystem.ultrasonicSensor.getDistance()));
			System.out.println(" UltrasonicSensor inches:" + Math.abs(this.ultrasonicSensorDistance));
//			System.out.println("ending drive to encoder");
			return true;
			
		}
		return false;
	}
}
