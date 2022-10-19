package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterLogInputs implements LoggableInputs {
    private static ShooterLogInputs INSTANCE = null;
    public double velocityRpm;
    public double setpointRpm;
    public double current;

    public static ShooterLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterLogInputs();
        }
        return INSTANCE;
    }

    private ShooterLogInputs() {
    }

    @Override
    public void toLog(LogTable table) {
        table.put("VelocityRPM", velocityRpm);
        table.put("SetpointRPM", setpointRpm);
        table.put("Current", current);
    }

    @Override
    public void fromLog(LogTable table) {
        velocityRpm = table.getDouble("VelocityRPM", velocityRpm);
        setpointRpm = table.getDouble("SetpointRPM", setpointRpm);
        current = table.getDouble("Current", current);
    }
}
