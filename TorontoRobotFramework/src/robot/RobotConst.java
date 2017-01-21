package robot;
/**
 * The RobotConst is a set of Robot constants that are determined by measuring
 * the ouput of the robot.
 */
public class RobotConst {

	/** 
	 * Max Drive Speed is the maximum encoder feedback rate that is received from the 
	 * drive encoders with an unloaded robot (robot on blocks).  This represents the 
	 * theoretical maximum speed of the robot.
	 */
    public static final double MAX_DRIVE_SPEED = 1800;
    
    /** The measured encoder counts per foot */
    public static final double DRIVE_ENCODER_COUNTS_PER_FT = 300.0;
    
    /** Gyro PID proportional gain */
    public static final double GYRO_PROPORTIONAL_GAIN = .05; // % per degree.
    
    /** Drive PID proportional gain */
    public static final double DRIVE_PID_PROPORTIONAL_GAIN = 0.50;
    
    /** Gyro Sensitivity for calibration of the rotational rate of the gyro */
    public static final double GYRO_SENSITIVITY = .0011;
    
}
