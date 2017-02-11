
package robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import robot.Robot;
import robot.commands.auto.DriveToEncoderDistanceCommand;
import robot.commands.auto.RotateToHeadingCommand;

/**
 *
 */
public class GearCommand extends Command {

	// enum gearState { OPEN, CLOSED };

	// gearState robotGearState = gearState.OPEN;

	public GearCommand() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.gearSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.gearSubsystem.close();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.gearSubsystem.getCurrentState() == true && Robot.chassisSubsystem.getSpeed() > 0.2) {
			Robot.gearSubsystem.close();
			return;
		}

		// switch (robotGearState) {
		// case OPEN:
		// if (Robot.gearSubsystem.getCurrentState() == true &&
		// Robot.chassisSubsystem.getSpeed() > 0.2) {
		// Robot.gearSubsystem.close();
		// return;
		// }
		// break;
		// case CLOSED:
		// if (Robot.gearSubsystem.getCurrentState() == false) {
		// if (Robot.chassisSubsystem.atTower() == true) {
		// Robot.gearSubsystem.open();
		// }
		// return;
		// }
		// break;
		// }

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timeSinceInitialized() > 2000;
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
	}
}
