package frc.robot.utils.log;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LoggedSubsystem;

import java.util.function.BooleanSupplier;

public class KnownIssue extends LoggedSubsystem {
    private final BooleanSupplier trigger;
    private final String name;
    private final String cause;
    private final String solution;

    public KnownIssue(BooleanSupplier trigger, String name, String cause, String solution) {
        super(new KnownIssueLogInputs(
                "Cause : " + cause + "\n" +
                        "Solution : " + solution + "\n", name, trigger));
        this.trigger = trigger;
        this.name = name;
        this.cause = cause;
        this.solution = solution;
    }

    @Override
    public String toString() {
        return "Cause : " + cause + "\n" +
                "Solution : " + solution + "\n";
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putString("KnownIssue : " + name, trigger.getAsBoolean() ? toString() : "");
    }

    @Override
    public String getSubsystemName() {
        return "KnownIssue : " + name;
    }
}
