
package robot.subsystems;

import com.toronto.pid.T_MotorSpeedPidController;
import com.toronto.sensors.T_Encoder;
import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.commands.T_DefaultDriveCommand;

public abstract class T_DriveSubsystem extends T_Subsystem {

	/* ****************************************************************************
	 * Hardware declarations
	 * 
	 * Declare all motors and sensors here
	 ******************************************************************************/
	private final SpeedController leftMotor;
	private final SpeedController leftMotor2;
	private final SpeedController rightMotor;
	private final SpeedController rightMotor2;

	private final T_Encoder leftEncoder;
	private final T_Encoder rightEncoder;

	private final double encoderCountsPerInch;
	
	T_MotorSpeedPidController leftMotorPidController;
	T_MotorSpeedPidController rightMotorPidController;

	private boolean drivePidsEnabled = false;

	public T_DriveSubsystem(SpeedController leftMotor, SpeedController rightMotor,
			T_Encoder leftEncoder, T_Encoder rightEncoder, double encoderCountsPerInch) {
		this(leftMotor, null, rightMotor, null, leftEncoder, rightEncoder, encoderCountsPerInch);
	}
	
	public T_DriveSubsystem(SpeedController leftMotor, SpeedController leftMotor2,
			SpeedController rightMotor, SpeedController rightMotor2,
			T_Encoder leftEncoder, T_Encoder rightEncoder, double encoderCountsPerInch) {
		
		this.leftMotor    = leftMotor;
		this.leftMotor2   = leftMotor2;
		this.rightMotor   = rightMotor;
		this.rightMotor2  = rightMotor2;
		
		this.leftEncoder  = leftEncoder;
		this.rightEncoder = rightEncoder;
		
		this.encoderCountsPerInch = encoderCountsPerInch;
		
	}
	
	/* ****************************************************************************
	 * Put methods for controlling this subsystem here.
	 * Call these from Commands.
	 ******************************************************************************/

	public void initDefaultCommand() {
		setDefaultCommand(new T_DefaultDriveCommand());
	}

	protected SpeedController getLeftMotor()   { return leftMotor; }
	protected SpeedController getLeftMotor2()  { return leftMotor2; }
	protected SpeedController getRightMotor()  { return rightMotor; }
	protected SpeedController getRightMotor2() { return rightMotor2; }
	protected T_Encoder getLeftEncoder()       { return leftEncoder; }
	protected T_Encoder getRightEncoder()      { return rightEncoder; }
	/**
	 * Get the speed of both encoders
	 * @return encoder counts per second
	 */
	public double getEncoderSpeed() {
		return ((leftEncoder.getRate() + rightEncoder.getRate())) / 2;
	}
	
	/**
	 * Get the distance in encoder counts
	 * @return encoder counts distance
	 */
	public double getEncoderDistance() {
		return ((leftEncoder.get() + rightEncoder.get())) / 2;
	}
	
	/**
	 * Get the distance in encoder counts
	 * @return encoder counts distance
	 */
	public double getDistanceInches() {
		return ((leftEncoder.get() + rightEncoder.get())) / 2 / encoderCountsPerInch;
	}
	
	/**
	 * Get the average speed of the drive base
	 * @return inches per second
	 */
	public double getSpeedInchesPerSecond() {
		return ((leftEncoder.getRate() + rightEncoder.getRate())) / 2 / encoderCountsPerInch;
	}
	
	public void enableDrivePids() {

		if (! leftMotorPidController.isEnabled()) {
			leftMotorPidController.enable();
		}
		if (! rightMotorPidController.isEnabled()) {
			rightMotorPidController.enable();
		}
		drivePidsEnabled = true;
	}

	public void disableDrivePids() {

		if (leftMotorPidController.isEnabled()) {
			leftMotorPidController.disable();
		}
		if (rightMotorPidController.isEnabled()) {
			rightMotorPidController.disable();
		}
		drivePidsEnabled = false;
	}

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

		if (drivePidsEnabled) {
			leftMotorPidController .setSetpoint(leftSpeed);
			rightMotorPidController.setSetpoint(rightSpeed);

			leftMotor .set(leftMotorPidController .get());
			if (leftMotor2 != null) {
				leftMotor2.set(leftMotorPidController.get());
			}
			rightMotor.set(rightMotorPidController.get());
			if (rightMotor2 != null) {
				rightMotor2.set(rightMotorPidController.get());
			}
		}
		else {
			leftMotor .set(leftSpeed);
			if (leftMotor2 != null) {
				leftMotor2.set(leftSpeed);
			}
			rightMotor.set(rightSpeed);
			if (rightMotor2 != null) {
				rightMotor2.set(rightSpeed);
			}
		}
	}

	public void robotInit() {
		enableDrivePids();
	}

	public void resetEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
	}

	@Override
	public void updatePeriodic() {

		if (drivePidsEnabled) {
			// Calculate all PIDs (only once per loop)
			leftMotorPidController.calculatePidOutput();
			rightMotorPidController.calculatePidOutput();

			leftMotor .set(leftMotorPidController .get());
			rightMotor.set(rightMotorPidController.get());
		}
		
		// Update all SmartDashboard values
		SmartDashboard.putNumber("Left Encoder Counts",     leftEncoder.get());
		SmartDashboard.putNumber("Left Encoder Raw Speed",  leftEncoder.getRate());
		SmartDashboard.putNumber("Right Encoder Counts",    rightEncoder.get());
		SmartDashboard.putNumber("Right Encoder Raw Speed", rightEncoder.getRate());
		
		SmartDashboard.putNumber("Robot Encoder Distance",  getEncoderDistance());
		SmartDashboard.putNumber("Robot Distance",          getDistanceInches());
		SmartDashboard.putNumber("Robot Encoder Speed",     getEncoderSpeed());
		SmartDashboard.putNumber("Robot Speed",             getSpeedInchesPerSecond());

		SmartDashboard.putData("Left Motor PID", leftMotorPidController);
		SmartDashboard.putData("Right Motor PID", rightMotorPidController);
		SmartDashboard.putNumber("Left Motor Error", leftMotorPidController.getError());
		SmartDashboard.putNumber("Right Motor Error", rightMotorPidController.getError());
		SmartDashboard.putNumber("Left Motor Output", leftMotorPidController.get());
		SmartDashboard.putNumber("Right Motor Output", rightMotorPidController.get());

	}

}

