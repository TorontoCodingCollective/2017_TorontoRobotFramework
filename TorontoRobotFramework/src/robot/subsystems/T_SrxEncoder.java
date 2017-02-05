package robot.subsystems;

import com.ctre.CANTalon;
import com.toronto.sensors.T_Encoder;

public class T_SrxEncoder extends T_Encoder {

	private final CANTalon canTalon;
	
	public T_SrxEncoder(CANTalon canTalon) {
		this(canTalon, false);
	}
	
	public T_SrxEncoder(CANTalon canTalon, boolean inverted) {
		super(inverted);
		this.canTalon = canTalon;
	}
	
	@Override
	public int get() {
		if (super.inverted) {
			return -canTalon.getEncPosition();
		}
		else {
			return canTalon.getEncPosition();
		}
	}

	@Override
	public double getRate() {
		if (super.inverted) {
			return -canTalon.getEncVelocity();
		}
		else {
			return canTalon.getEncVelocity();
		}
	}

	@Override
	public void reset() {
		canTalon.setEncPosition(0);
	}

}
