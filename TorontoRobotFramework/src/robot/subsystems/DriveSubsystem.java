
package robot.subsystems;

import com.toronto.pid.T_GyroPidController;
import com.toronto.sensors.T_DioEncoder;
import com.toronto.sensors.T_Gyro;
import com.toronto.sensors.T_LimitSwitch;
import com.toronto.sensors.T_LimitSwitch.DefaultState;
import com.toronto.sensors.T_UltrasonicSensor;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.RobotConst;
import robot.RobotMap;
import robot.commands.JoystickCommand;

public class DriveSubsystem extends T_DriveSubsystem {

	/* ****************************************************************************
	 * Hardware declarations
	 * 
	 * Declare all motors and sensors here
	 ******************************************************************************/
	private T_Gyro gyro = new T_Gyro(RobotMap.GYRO_ANALOG_INPUT_PORT, RobotConst.INVERTED);

	T_GyroPidController gyroPidController = 
			new T_GyroPidController(RobotConst.GYRO_PID_PROPORTIONAL_GAIN, RobotConst.GYRO_PID_INTEGRAL_GAIN, gyro);

	public T_LimitSwitch towerSensor = new T_LimitSwitch(RobotMap.FRONT_LIMIT_SWITCH_DIO_PORT, DefaultState.TRUE);
	public T_UltrasonicSensor ultrasonicSensor = new T_UltrasonicSensor(1);

	public Solenoid shifterSolenoid = new Solenoid(RobotMap.SHIFTER_SOLENOID);
	
	public DriveSubsystem() {
		
		super(  new VictorSP(RobotMap.LEFT_MOTOR_PWM_PORT), 
				new VictorSP(RobotMap.RIGHT_MOTOR_PWM_PORT),
				new T_DioEncoder(RobotMap.LEFT_ENCODER_A_DIO_PORT,  RobotMap.LEFT_ENCODER_B_DIO_PORT),
				new T_DioEncoder(RobotMap.RIGHT_ENCODER_A_DIO_PORT, RobotMap.RIGHT_ENCODER_B_DIO_PORT, RobotConst.INVERTED),
				RobotConst.DRIVE_ENCODER_COUNTS_PER_IN
				);
		
		// Invert the right motors
		super.getRightMotor() .setInverted(true);
		if (super.getRightMotor2() != null) {
			super.getRightMotor2().setInverted(true);
		}
	}

	/* ****************************************************************************
	 * Put methods for controlling this subsystem here.
	 * Call these from Commands.
	 ******************************************************************************/

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickCommand());
	}

	public boolean atTower() {
		return towerSensor.atLimit();
	}
	
	public T_LimitSwitch getTowerSensor() {
		return towerSensor;
	}
	
	public void setHighGear() {
		shifterSolenoid.set(false);
	}

	public void setLowGear() {
		shifterSolenoid.set(true);
	}

	public void robotInit() {

		gyro.initGyro();
		gyro.setSensitivity(RobotConst.GYRO_SENSITIVITY);
		gyroPidController.disable();

		// Calibrate the ultrasonic
		ultrasonicSensor.calibrate(RobotConst.ULTRASONIC_VOLTAGE_20IN, RobotConst.ULTRASONIC_VOLTAGE_40IN, RobotConst.ULTRASONIC_VOLTAGE_80IN);

		setLowGear();
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


	@Override
	public void updatePeriodic() {

		SmartDashboard.putData("Gyro", gyro);
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate",  gyro.getRate());
		SmartDashboard.putData("Gyro PID", gyroPidController);
		SmartDashboard.putNumber("Gyro PID Error", gyroPidController.getError());
		SmartDashboard.putNumber("Gyro PID Output", gyroPidController.get());

		SmartDashboard.putBoolean("Tower Sensor", towerSensor.atLimit());

		SmartDashboard.putNumber("Ultrasonic voltage", ultrasonicSensor.getVoltage());
		SmartDashboard.putNumber("Ultrasonic distance", ultrasonicSensor.getDistance());
	}

}

