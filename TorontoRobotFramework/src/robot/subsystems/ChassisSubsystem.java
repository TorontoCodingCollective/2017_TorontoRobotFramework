
package robot.subsystems;

import com.toronto.pid.T_GyroPidController;
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

	private AnalogInput ultrasonicSensor = new AnalogInput(3);

	/* ****************************************************************************
	 * Put methods for controlling this subsystem here.
	 * Call these from Commands.
	 ******************************************************************************/

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickCommand());
	}

	public void setBothMotorSpeeds(double speed) {
		leftMotor .set(calcPIDValue(speed, leftEncoder.getRate()));
		rightMotor.set(calcPIDValue(speed, rightEncoder.getRate()));
	}

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
		//    	leftMotor.set(leftSpeed);
		//    	rightMotor.set(rightSpeed);
		leftMotor .set(calcPIDValue(leftSpeed, leftEncoder.getRate()));
		rightMotor.set(calcPIDValue(rightSpeed, rightEncoder.getRate()));
	}

	public void setMotorSpeed(double forwardSpeed, double turnSpeed) {
		if (Math.abs(forwardSpeed) <= 0.03) {
			leftMotor.set(turnSpeed);
			rightMotor.set(-turnSpeed);
		}
	}

	public void robotInit() {
		rightMotor.setInverted(true);

		gyro.initGyro();
		gyro.setSensitivity(RobotConst.GYRO_SENSITIVITY);
		gyroPidController.disable();
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
		return gyroPidController.calculatePidOutput();
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
		return (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;
	}

	private double calcPIDValue(double setPoint, double feedback) {

		double normalizedFeedback = feedback/RobotConst.MAX_DRIVE_SPEED;
		if (normalizedFeedback >  1.0) { normalizedFeedback =  1.0; }
		if (normalizedFeedback < -1.0) { normalizedFeedback = -1.0; }

		double error = setPoint - normalizedFeedback;

		double output = setPoint + error * RobotConst.DRIVE_PID_PROPORTIONAL_GAIN;

		double normalizedOutput = output;

		if (normalizedOutput >  1.0) { normalizedOutput =  1.0; }
		if (normalizedOutput < -1.0) { normalizedOutput = -1.0; }

		return normalizedOutput;
	}

	@Override
	public void updatePeriodic() {
		
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
	}
}

