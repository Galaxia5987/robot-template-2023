package frc.robot.utils.log;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.function.BooleanSupplier;

public class KnownIssueLogInputs implements LoggableInputs {
    public final BooleanSupplier trigger;
    public final String displayName;
    public final String description;

    public KnownIssueLogInputs(String description, String displayName, BooleanSupplier trigger) {
        this.displayName = displayName;
        this.description = description;
        this.trigger = trigger;
    }

    @Override
    public void toLog(LogTable table) {
        table.put(displayName, trigger.getAsBoolean() ? description : "");
    }

    @Override
    public void fromLog(LogTable table) {}
}
