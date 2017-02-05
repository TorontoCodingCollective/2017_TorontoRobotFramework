
package robot.subsystems;

import com.ctre.CANTalon;
import com.toronto.pid.T_GyroPidController;
import com.toronto.pid.T_MotorSpeedPidController;
import com.toronto.sensors.T_DioEncoder;
import com.toronto.sensors.T_Encoder;
import com.toronto.sensors.T_Gyro;
import com.toronto.subsystems.T_Subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
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
	private SpeedController leftMotor;
	private SpeedController rightMotor;
	
	private T_Encoder leftEncoder;
	private T_Encoder rightEncoder;
	
	private T_Gyro gyro = new T_Gyro(RobotMap.GYRO_ANALOG_INPUT_PORT, RobotConst.INVERTED);

	T_GyroPidController gyroPidController = 
			new T_GyroPidController(RobotConst.GYRO_PID_PROPORTIONAL_GAIN, RobotConst.GYRO_PID_INTEGRAL_GAIN, gyro);

	T_MotorSpeedPidController leftMotorPidController;
	T_MotorSpeedPidController rightMotorPidController;

	private boolean drivePidsEnabled = false;
	
	public AnalogInput ultrasonicSensor = new AnalogInput(1);

	public ChassisSubsystem() {
		
		// Use the robot number to determine which type of motor drive to use.  The
		// 1320 robot uses the PWM Victor controllers
		// 1321 uses CAN bus TalonSRX controllers.
		
		if (RobotConst.ROBOT == 1321) {
	
			// Use the CAN bus motor controllers and encoders
			leftMotor  = new CANTalon(RobotMap.LEFT_MOTOR_CAN_ADDRESS);
			rightMotor = new CANTalon(RobotMap.RIGHT_MOTOR_CAN_ADDRESS);
			rightMotor.setInverted(true);
			
			leftEncoder  = new T_SrxEncoder((CANTalon) leftMotor);
			rightEncoder = new T_SrxEncoder((CANTalon) rightMotor);
			
		} else {

			// Use the PWM bus motor controllers and encoders
			leftMotor  = new VictorSP(RobotMap.LEFT_MOTOR_PWM_PORT);
			rightMotor = new VictorSP(RobotMap.RIGHT_MOTOR_PWM_PORT);
			rightMotor.setInverted(true);
			
			leftEncoder = 
					new T_DioEncoder(RobotMap.LEFT_ENCODER_A_DIO_PORT,  RobotMap.LEFT_ENCODER_B_DIO_PORT);
			rightEncoder = 
					new T_DioEncoder(RobotMap.RIGHT_ENCODER_A_DIO_PORT, RobotMap.RIGHT_ENCODER_B_DIO_PORT, RobotConst.INVERTED);

		}
		
		// Once the motor and encoders are set, the speed controllers are identical
		leftMotorPidController = 
				new T_MotorSpeedPidController(RobotConst.DRIVE_PID_PROPORTIONAL_GAIN, RobotConst.DRIVE_PID_INTEGRAL_GAIN, 
					leftEncoder, RobotConst.DRIVE_ENCODER_MAX_SPEED);

		rightMotorPidController = 
				new T_MotorSpeedPidController(RobotConst.DRIVE_PID_PROPORTIONAL_GAIN, RobotConst.DRIVE_PID_INTEGRAL_GAIN, 
					rightEncoder, RobotConst.DRIVE_ENCODER_MAX_SPEED);
		
	}
	
	/* ****************************************************************************
	 * Put methods for controlling this subsystem here.
	 * Call these from Commands.
	 ******************************************************************************/

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickCommand());
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
			rightMotor.set(rightMotorPidController.get());
		}
		else {
			leftMotor .set(leftSpeed);
			rightMotor.set(rightSpeed);
		}
	}

	public void robotInit() {

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
		return (rightEncoder.get() + leftEncoder.get()) / (RobotConst.DRIVE_ENCODER_COUNTS_PER_IN * 2);
	}

	@Override
	public void updatePeriodic() {

		// Calculate all PIDs (only once per loop)
		leftMotorPidController.calculatePidOutput();
		rightMotorPidController.calculatePidOutput();
		gyroPidController.calculatePidOutput();
		
		// Update all SmartDashboard values
		SmartDashboard.putNumber("Left Encoder Distance",  leftEncoder.get());
		SmartDashboard.putNumber("Left Encoder Speed",     leftEncoder.getRate());
		SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.get());
		SmartDashboard.putNumber("Right Encoder Speed",    rightEncoder.getRate());
		
		SmartDashboard.putData("Left Motor PID", leftMotorPidController);
		SmartDashboard.putData("Right Motor PID", rightMotorPidController);
		SmartDashboard.putNumber("Left Motor Setpoint", leftMotorPidController.getSetpoint());
		SmartDashboard.putNumber("Right Motor Setpoint", rightMotorPidController.getSetpoint());
		SmartDashboard.putNumber("Left Motor Error", leftMotorPidController.getError());
		SmartDashboard.putNumber("Right Motor Error", rightMotorPidController.getError());
		SmartDashboard.putNumber("Left Motor Output", leftMotorPidController.get());
		SmartDashboard.putNumber("Right Motor Output", rightMotorPidController.get());
		
		SmartDashboard.putData("Gyro", gyro);
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate",  gyro.getRate());
		SmartDashboard.putData("Gyro PID", gyroPidController);
		SmartDashboard.putNumber("Gyro PID Error", gyroPidController.getError());
		SmartDashboard.putNumber("Gyro PID Output", gyroPidController.get());

		SmartDashboard.putNumber("Raw Ultrasonic Value", ultrasonicSensor.getVoltage());

}
}

