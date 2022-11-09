package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;

public abstract class LoggedSubsystem extends SubsystemBase {
    public static final String subsystemsDirectory = "subsystems/";
    private static final List<LoggedSubsystem> subsystems = new ArrayList<>();

    public LoggedSubsystem() {
        subsystems.add(this);
    }

    public static List<LoggedSubsystem> getSubsystems() {
        return subsystems;
    }

    public abstract void log();

    public abstract String getSubsystemName();

    public String getLogDirectory(String name) {
        return subsystemsDirectory + getSubsystemName() + "/" + name;
    }

    public String getMotorDirectory(String motorLogName) {
        return getLogDirectory("motors") + "/" + motorLogName;
    }
}
