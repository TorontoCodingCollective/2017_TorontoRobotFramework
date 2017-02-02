package robot.oi;

import com.toronto.oi.T_Axis;
import com.toronto.oi.T_Button;
import com.toronto.oi.T_Logitech_GameController;
import com.toronto.oi.T_OiController;
import com.toronto.oi.T_Stick;
import com.toronto.oi.T_Toggle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * Operator Controller (Game Controller)
 * -------------------------------------
 * Joysticks
 * ---------
 * Left:        Turn - X Axis
 * Right:       Speed - Y Axis
 * 
 * Triggers
 * ---------
 * Left:
 * Right:
 * 
 * Buttons
 * ---------
 * A:
 * B:
 * X:
 * Y:			Drive Straight Command
 * 
 * LBumper:
 * RBumper: 	Rumble (test) the Driver Controller
 * Start:		
 * Back:		Toggle (test)
 * 
 * POV:       	RotateToAngle
 *
 */
public class OI {
	
	public AutoSelector autoSelector = new AutoSelector();

	private T_OiController driverController = new T_Logitech_GameController(0);
	
	private T_Toggle driverTestToggle = new T_Toggle(driverController, T_Button.BACK);
	
	public boolean getDriverRumbleStart() {
		return driverController.getButton(T_Button.RIGHT_BUMPER);
	}
	
	public double getSpeed() {
		return driverController.getAxis(T_Stick.RIGHT, T_Axis.Y);
	}
	
	public boolean getStartDriveStraightCommand() {
		return driverController.getButton(T_Button.Y);
	}
	
	public boolean getDriverToggle() {
		return driverTestToggle.getValue();
	}
	
	public int getRotateToAngle() {
		return driverController.getPov();
	}
	
	public double getTurn() {
		return driverController.getAxis(T_Stick.LEFT, T_Axis.X);
	}

	public boolean isDriverAction() {
		return driverController.isControllerActivated();
	}

	public void setDriverRumble(double rumble) {
		driverController.setRumble(rumble);
	}

	public boolean getCancel() {
		return driverController.getButton(T_Button.BACK);
	}
	
	public boolean getCalibrate() {
		return driverController.getButton(T_Button.START);
	}
	
	public void teleopPeriodic() {
		driverTestToggle.update();
	}
	
	public void updatePeriodic() {
		
		autoSelector.updateSmartDashboard();
		SmartDashboard.putString("Driver Controller", 
				driverController.toString());
		SmartDashboard.putBoolean("Toggle", getDriverToggle());
	}
}

