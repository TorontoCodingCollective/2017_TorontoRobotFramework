
package robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.RobotConst;
import robot.RobotMap;
import robot.commands.T_GameControllerDriveCommand;

public class ChassisSubsystem extends Subsystem {

	/* ****************************************************************************
	 * Hardware declarations
	 * 
	 * Declare all motors and sensors here
	 ******************************************************************************/
	private Victor leftMotor  = new Victor(RobotMap.LEFT_MOTOR);
	private Victor rightMotor = new Victor(RobotMap.RIGHT_MOTOR);
	
	private Encoder leftEncoder = 
			new Encoder(RobotMap.LEFT_ENCODER_A, RobotMap.LEFT_ENCODER_B);
	private Encoder rightEncoder = 
			new Encoder(RobotMap.RIGHT_ENCODER_A, RobotMap.RIGHT_ENCODER_B, true);
	
	private AnalogGyro gyro = new AnalogGyro(RobotMap.GYRO) {
		@Override
		public double getAngle() {
			return -super.getAngle();
		}
	};

	/* ****************************************************************************
     * Put methods for controlling this subsystem here.
     * Call these from Commands.
	 ******************************************************************************/

    public void initDefaultCommand() {
        setDefaultCommand(new T_GameControllerDriveCommand());
    }
    
    public void setBothMotorSpeeds(double speed) {
    	leftMotor .set(calcPIDValue(speed, leftEncoder.getRate()));
    	rightMotor.set(calcPIDValue(speed, rightEncoder.getRate()));
    }
    
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
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
	}

	public void resetEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
	}
	
	public double getAngle() {
		return gyro.getAngle() % 360.0;
	}
	
	public double getEncoderDistance() {
		return (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;
	}
	
	public void updateSmartDashboard() {
		SmartDashboard.putData("Left Encoder", leftEncoder);
		SmartDashboard.putData("Right Encoder", rightEncoder);
		SmartDashboard.putData("Gyro", gyro);
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
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
}

