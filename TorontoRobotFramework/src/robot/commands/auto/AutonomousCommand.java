package robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;
import robot.oi.AutoSelector.BoilerPosition;
import robot.oi.AutoSelector.RobotPosition;
import robot.oi.AutoSelector.ShootMode;

/**
 * This is the auto command that will use the operator input to build the correct command
 */
public class AutonomousCommand extends CommandGroup {

    public AutonomousCommand() {
    	
    	RobotPosition  robotPosition  = Robot.oi.autoSelector.getRobotPostion();
    	BoilerPosition boilerPosition = Robot.oi.autoSelector.getBoilerPostion();
    	ShootMode      shootMode      = Robot.oi.autoSelector.getShootMode();
    	
    	System.out.println("Robot Position " + robotPosition);
    	System.out.println("Boiler Position " + boilerPosition);
    	System.out.println("Shoot Mode " + shootMode);
    	
    	
    	if (robotPosition == RobotPosition.CENTER) {
    		addSequential(new DriveToEncoderDistanceCommand(0, .6, 77.0));
    	}
    	if (robotPosition == RobotPosition.LEFT) {
    		addSequential(new DriveToEncoderDistanceCommand(0, .5, 96));
    		addSequential(new RotateToHeadingCommand(60));
    		addSequential(new DriveToEncoderDistanceCommand(60, .2, 30));
    	}
    	if (robotPosition == robotPosition.RIGHT) {
    		addSequential(new DriveToEncoderDistanceCommand(0, .5, 86.5));
    		addSequential(new RotateToHeadingCommand(302));
    		addSequential(new DriveToEncoderDistanceCommand(302, .3, 32.0));
    	}
    
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
