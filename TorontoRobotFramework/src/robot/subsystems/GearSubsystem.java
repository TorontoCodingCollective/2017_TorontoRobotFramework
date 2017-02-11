
package robot.subsystems;

import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.GearCommand;
import robot.commands.JoystickCommand;

public class GearSubsystem extends T_Subsystem {

	/*
	 * *************************************************************************
	 * *** Hardware declarations
	 * 
	 * Declare all motors and sensors here
	 ******************************************************************************/
	private Solenoid release = new Solenoid(0);

	public void initDefaultCommand() {
		setDefaultCommand(new GearCommand());
	}
	
	public void open() { 
		release.set(false);
	}

	public void close() {
		release.set(true);
	}

	public boolean getCurrentState() {
		return release.get();
	}

	@Override
	public void updatePeriodic() {
		// Update all SmartDashboard values
		SmartDashboard.putString("Gear State", release.get() ? "Gear Locked" : "Gear Released");
	}

	@Override
	public void robotInit() {
		release.set(true);
	}
}
