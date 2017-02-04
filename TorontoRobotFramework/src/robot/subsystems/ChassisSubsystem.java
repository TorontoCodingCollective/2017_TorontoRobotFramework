
package robot.subsystems;

import com.toronto.pid.T_GyroPidController;
import com.toronto.pid.T_MotorSpeedPidController;
import com.toronto.sensors.T_Gyro;
import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.RobotConst;
import robot.RobotMap;
import robot.commands.JoystickCommand;

public class ChassisSubsystem extends T_Subsystem {

	/* ****************************************************************************
	 * Hardware declarations
	 * 
	 * Declare all motors and sensors here
	 ******************************************************************************/
	private Victor leftMotor  = new Victor(RobotMap.LEFT_MOTOR_PWM_PORT);
	private Victor rightMotor = new Victor(RobotMap.RIGHT_MOTOR_PWM_PORT);

	private Encoder leftEncoder = 
			new Encoder(RobotMap.LEFT_ENCODER_A_DIO_PORT,  RobotMap.LEFT_ENCODER_B_DIO_PORT);
	private Encoder rightEncoder = 
			new Encoder(RobotMap.RIGHT_ENCODER_A_DIO_PORT, RobotMap.RIGHT_ENCODER_B_DIO_PORT, RobotConst.INVERTED);

	private T_Gyro gyro = new T_Gyro(RobotMap.GYRO_ANALOG_INPUT_PORT, RobotConst.INVERTED);

	T_GyroPidController gyroPidController = 
			new T_GyroPidController(RobotConst.GYRO_PROPORTIONAL_GAIN, RobotConst.GYRO_INTEGRAL_GAIN, gyro);

	T_MotorSpeedPidController leftMotorSpeedController = 
			new T_MotorSpeedPidController(RobotConst.DRIVE_PID_PROPORTIONAL_GAIN, RobotConst.DRIVE_PID_INTEGRAL_GAIN, 
					leftEncoder, RobotConst.MAX_DRIVE_ENCODER_SPEED);

	T_MotorSpeedPidController rightMotorSpeedController = 
			new T_MotorSpeedPidController(RobotConst.DRIVE_PID_PROPORTIONAL_GAIN, RobotConst.DRIVE_PID_INTEGRAL_GAIN, 
					rightEncoder, RobotConst.MAX_DRIVE_ENCODER_SPEED);

	private AnalogInput ultrasonicSensor = new AnalogInput(3);

	private boolean drivePidsEnabled = false;
	/* ****************************************************************************
	 * Put methods for controlling this subsystem here.
	 * Call these from Commands.
	 ******************************************************************************/

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickCommand());
	}

	public void enableDrivePids() {
		if (! leftMotorSpeedController.isEnabled()) {
			leftMotorSpeedController.enable();
		}
		if (! rightMotorSpeedController.isEnabled()) {
			rightMotorSpeedController.enable();
		}
		drivePidsEnabled = true;
	}
	
	public void disableDrivePids() {
		if (leftMotorSpeedController.isEnabled()) {
			leftMotorSpeedController.disable();
		}
		if (rightMotorSpeedController.isEnabled()) {
			rightMotorSpeedController.disable();
		}
		drivePidsEnabled = false;
	}

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
		
		if (drivePidsEnabled) {
			leftMotorSpeedController .setSetpoint(leftSpeed);
			rightMotorSpeedController.setSetpoint(rightSpeed);
			
			leftMotor .set(leftMotorSpeedController .get());
			rightMotor.set(rightMotorSpeedController.get());
		}
		else {
			leftMotor .set(leftSpeed);
			rightMotor.set(rightSpeed);
		}
	}

	public void robotInit() {
		rightMotor.setInverted(true);

		gyro.initGyro();
		gyro.setSensitivity(RobotConst.GYRO_SENSITIVITY);
		gyroPidController.disable();
		
		enableDrivePids();
	}

	public void resetEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
	}

	public void enableGyroPid() {
		if (!gyroPidController.isEnabled()) {
			gyroPidController.enable();
		}
	}

	public void calibrateGyro() {
		gyro.reset();
		gyro.calibrate();
	}

	public void disableGyroPid() {
		if (gyroPidController.isEnabled()) {
			gyroPidController.disable();
		}
	}

	public double getGyroPidOutput() {
		return gyroPidController.get();
	}

	public void setGyroPidSetpoint(double angle) {
		gyroPidController.setSetpoint(angle);
	}

	public double getAngle() {
		return gyro.getAngle();
	}

	public double getAngleRate() {
		return gyro.getRate();
	}

	public double getAngleError(double heading) {

		// FIXME:  Move this code to the T_GYRO class
		// return gyro.getAngleError(heading);

		double angleError = heading - getAngle();

		if (angleError > 180.0)  { angleError -= 360.0; }
		if (angleError < -180.0) { angleError += 360.0; }

		return angleError;
	}

	public double getEncoderDistanceInches() {
		return (rightEncoder.getDistance() + leftEncoder.getDistance()) / (RobotConst.DRIVE_ENCODER_COUNTS_PER_IN * 2);
	}

	@Override
	public void updatePeriodic() {

		// Calculate all PIDs (only once per loop)
		leftMotorSpeedController.calculatePidOutput();
		rightMotorSpeedController.calculatePidOutput();
		gyroPidController.calculatePidOutput();
		
		// Update all SmartDashboard values
		SmartDashboard.putData("Left Encoder", leftEncoder);
		SmartDashboard.putData("Right Encoder", rightEncoder);
		SmartDashboard.putData("Gyro", gyro);
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate",  gyro.getRate());
		SmartDashboard.putNumber("Gyro PID Error", gyroPidController.getError());
		SmartDashboard.putNumber("Gyro PID Output", gyroPidController.get());
		SmartDashboard.putNumber("Raw Ultrasonic Value", ultrasonicSensor.getValue());
		SmartDashboard.putData("Gyro PID", gyroPidController);
		SmartDashboard.putNumber("Left Motor Setpoint", leftMotorSpeedController.getSetpoint());
		SmartDashboard.putNumber("Right Motor Setpoint", rightMotorSpeedController.getSetpoint());
		SmartDashboard.putNumber("Left Motor Output", leftMotorSpeedController.get());
		SmartDashboard.putNumber("Right Motor Output", rightMotorSpeedController.get());
		SmartDashboard.putNumber("Left Motor Error", leftMotorSpeedController.getError());
		SmartDashboard.putNumber("Right Motor Error", rightMotorSpeedController.getError());
		SmartDashboard.putData("Left Motor PID", leftMotorSpeedController);
		SmartDashboard.putData("Right Motor PID", rightMotorSpeedController);
	}
}

