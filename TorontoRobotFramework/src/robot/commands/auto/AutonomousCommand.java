package robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.Robot;
import robot.commands.DefaultGearCommand;
import robot.commands.GearReleaseCommand;
import robot.oi.AutoSelector.BoilerPosition;
import robot.oi.AutoSelector.RobotPosition;
import robot.oi.AutoSelector.ShootMode;
import robot.subsystems.ChassisSubsystem;


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
    	
    	if (boilerPosition == BoilerPosition.RIGHT) {

        	if (robotPosition == RobotPosition.CENTER) {
        		
        		// Do Gear
           		addSequential(new DriveToLimitSwitchCommand(0, .4, Robot.chassisSubsystem.getTowerSensor(), 84));
        		addSequential(new GearReleaseCommand());
        		addSequential(new DriveToEncoderDistanceCommand(0, -.2, 24));
        		
        		if (shootMode == ShootMode.GEAR_SHOOT) {
	        		addSequential(new RotateToHeadingCommand(120));
	        		addSequential(new DriveToEncoderDistanceCommand(120, .6, 95));
        		}
        	}
        	
        	if (robotPosition == RobotPosition.LEFT) {
        		// Do Gear
        		addSequential(new DriveToEncoderDistanceCommand(0, .5, 94));
        		addSequential(new RotateToHeadingCommand(59));
    			addSequential(new DriveToLimitSwitchCommand(59, .2, Robot.chassisSubsystem.getTowerSensor(), 28));
    			addSequential(new GearReleaseCommand());
    			addSequential(new DriveToEncoderDistanceCommand(51, -.5, 25));
    			
    			// Since you won't be able to shoot from the right side, just drive back and straight
    			if (shootMode == ShootMode.GEAR_ONLY || shootMode == ShootMode.GEAR_SHOOT) {
        			addSequential(new RotateToHeadingCommand(0));
        			addSequential(new DriveToEncoderDistanceCommand(0, .5, 30));
        		}
        		
        		
        	}
        	if (robotPosition == RobotPosition.RIGHT) {
        		
        		// Do Gear
        		addSequential(new DriveToEncoderDistanceCommand(0, .5, 92));
        		addSequential(new RotateToHeadingCommand(300));
        		addSequential(new DriveToLimitSwitchCommand(300, .2, Robot.chassisSubsystem.getTowerSensor(), 32));
        		addSequential(new GearReleaseCommand());
        		addSequential(new DriveToEncoderDistanceCommand(300, -.5, 25));
        		
        		if (shootMode == ShootMode.GEAR_ONLY) {
        			addSequential(new RotateToHeadingCommand(0));
        			addSequential(new DriveToEncoderDistanceCommand(0, .5, 30));
        		}
        		
        		if (shootMode == ShootMode.GEAR_SHOOT) {
	        		addSequential(new RotateToHeadingCommand(145));
	        		addSequential(new DriveToEncoderDistanceCommand(145, .5, 60));
        		}
    			
    		
        	}
			
    	}
    	
    	if (boilerPosition == BoilerPosition.LEFT) {

        	if (robotPosition == RobotPosition.CENTER) {
        		
        		// Do Gear
           		addSequential(new DriveToLimitSwitchCommand(0, .4, Robot.chassisSubsystem.getTowerSensor(), 84));
        		addSequential(new GearReleaseCommand());
        		addSequential(new DriveToEncoderDistanceCommand(0, -.2, 24));
        		
        		if (shootMode == ShootMode.GEAR_SHOOT) {
	        		addSequential(new RotateToHeadingCommand(240));
	        		addSequential(new DriveToEncoderDistanceCommand(240, .6, 95));
        		}
        	}
        	
        	if (robotPosition == RobotPosition.LEFT) {
        	
        		// Do Gear
        		addSequential(new DriveToEncoderDistanceCommand(0, .5, 96));
        		addSequential(new RotateToHeadingCommand(60));
        		addSequential(new DriveToLimitSwitchCommand(60, .2, Robot.chassisSubsystem.getTowerSensor(), 38));
        		addSequential(new GearReleaseCommand());
        		addSequential(new DriveToEncoderDistanceCommand(60, -.5, 29));
        		
        		if (shootMode == ShootMode.GEAR_ONLY) {
        			addSequential(new RotateToHeadingCommand(0));
        			addSequential(new DriveToEncoderDistanceCommand(0, .5, 34));
        		}
        		
        		if (shootMode == ShootMode.GEAR_SHOOT) {
	        		addSequential(new RotateToHeadingCommand(215));
	        		addSequential(new DriveToEncoderDistanceCommand(215, .5, 64));
        		}
        		
        	}
        	if (robotPosition == RobotPosition.RIGHT) {
        		
        		// Do Gear
        		addSequential(new DriveToEncoderDistanceCommand(0, .5, 94));
        		addSequential(new RotateToHeadingCommand(301));
    			addSequential(new DriveToLimitSwitchCommand(301, .2, Robot.chassisSubsystem.getTowerSensor(), 28));
    			addSequential(new GearReleaseCommand());
    			addSequential(new DriveToEncoderDistanceCommand(301, -.5, 25));
    			
    			// Since you won't be able to shoot from the right side, just drive back and straight
    			if (shootMode == ShootMode.GEAR_ONLY || shootMode == ShootMode.GEAR_SHOOT) {
        			addSequential(new RotateToHeadingCommand(0));
        			addSequential(new DriveToEncoderDistanceCommand(0, .5, 30));
        		}
    			
    		
        	}
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
