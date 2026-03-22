package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SwagLights implements Subsystem {
    // Inputs and Outputs
    private final SerialPort serialPort;

    // Singleton Setup
    private static SwagLights instance;

    public static SwagLights getInstance() {
        if (instance == null) {
            instance = new SwagLights();
        }
        return instance;
    }

    /**
     * Initializes the serial communication
     */
    private SwagLights() {
        serialPort = new SerialPort(9600, SerialPort.Port.kMXP);
        serialPort.enableTermination();
        System.out.println("Created new serial reader");
    }

    @Override
    public void periodic() {
        LEDState activeState = LEDState.DISABLED;
        for (LEDState state : LEDState.values()) {
            if (state.isActive() && state.getPriority() >= activeState.getPriority()) {
                activeState = state;
            }
        }
        serialPort.writeString(String.valueOf(activeState.getValue()));
    }

    /**
     * Closes serial port listener
     */
    public void stopSerialPort() {
        System.out.println("Closing serial port");
        serialPort.close();
    }

    public enum LEDState {
        EMERGENCY_STOP('E', Integer.MAX_VALUE),
        TOO_CLOSE('C', 3),
        ALIGNED('S', 2),
        ALIGNING('A', 1),
        ENABLED('D', 0),
        DISABLED('D', 0);

        private final char value;
        private final int priority;
        private boolean active;

        LEDState(char value, int priority) {
            this.value = value;
            this.priority = priority;
            this.active = false;
        }

        public char getValue() {
            return value;
        }

        public int getPriority() {
            return priority;
        }

        public boolean isActive() {
            return active;
        }

        public void setActive(boolean active) {
            this.active = active;
        }
    }
}
