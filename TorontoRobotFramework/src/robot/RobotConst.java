package robot;
/**
 * The RobotConst is a set of Robot constants that are determined by measuring
 * the ouput of the robot.
 */
public class RobotConst {

	
	/** WpiLib compatible inverted indicator {@code boolean true} which can be 
	 * used to make the robot code more readable */
	public static final boolean INVERTED     = true;
	/** WpiLib compatible NOT inverted indicator {@code boolean false} which can be 
	 * used to make the robot code more readable */
	public static final boolean NOT_INVERTED = false;
	
	/** 
	 * Max Drive Speed is the maximum encoder feedback rate that is received from the 
	 * drive encoders with an unloaded robot (robot on blocks).  This represents the 
	 * theoretical maximum speed of the robot.
	 */
    public static final double MAX_DRIVE_ENCODER_SPEED = 1800;
    
    /** The measured encoder counts per in */
    public static final double DRIVE_ENCODER_COUNTS_PER_IN = 3570/200;
    
    /** Gyro PID proportional gain */
    public static final double GYRO_PROPORTIONAL_GAIN = 2.0;
    /** Gyro PID integral gain */
    public static final double GYRO_INTEGRAL_GAIN = .05;
    
    /** Gyro in-place pivot speed used when under gyro control */
    public static final double GYRO_PIVOT_SPEED = .65; 
    
    /** Drive PID proportional gain */
    public static final double DRIVE_PID_PROPORTIONAL_GAIN = 1.0;
    /** Drive PID integral gain */
    public static final double DRIVE_PID_INTEGRAL_GAIN = 0.0;
    
    /** Gyro Sensitivity for calibration of the rotational rate of the gyro */
    public static final double GYRO_SENSITIVITY = .00172;
    
}
