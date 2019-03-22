package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class SeatMotorCounter implements PIDSource{
    private int m_position;
    private Counter m_counter;
    public SeatMotorCounter(int port, Supplier<Integer> directionSupplier) {
        m_counter = new Counter(new DigitalInput(port));
        /*
        new Thread() {
            public void run() {
                while(!Thread.interrupted()) {
                    if (directionSupplier.get() ==  -1) {
                        m_position -= m_counter.get();
                    } else if (directionSupplier.get() == 1) {
                        m_position += m_counter.get();
                    }
                    m_counter.reset();
                }
            }
        }.start();
        */
    }

    public void calibrate(int calibrateTo) {
        m_position = calibrateTo;
    }
    
    public double getPosition() {
        return m_position;
    }

    public double getCounterValue() {
        return m_counter.get();
    }

    @Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return m_position;
	}
}