package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class T_GameControllerDriveCommand extends Command{
	
	public T_GameControllerDriveCommand(){
		requires(Robot.chassisSubsystem);
	}

	@Override
	protected void initialize() {

	}

	@Override
	protected void execute() {
		double speed = Robot.oi.getSpeed();
		double turn = Robot.oi.getTurn();
		if(Math.abs(turn) < 0.075){
			Robot.chassisSubsystem.setBothMotorSpeeds(speed);
		}else{
			if(Math.abs(speed) > 0.075 && turn > 0.075){
				Robot.chassisSubsystem.setMotorSpeeds(speed,0);
			}else if(Math.abs(speed) > 0.075 && turn < -0.075){
				Robot.chassisSubsystem.setMotorSpeeds(0,speed);
			}else{
				Robot.chassisSubsystem.setMotorSpeeds(turn, -turn);
			}
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
	}

}
