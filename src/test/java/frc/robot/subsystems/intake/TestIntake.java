package frc.robot.subsystems.intake;

import org.junit.After;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestIntake {
    private final Intake intake = Intake.getInstance();

    @Test
    public void setPower() {
        intake.setPower(1);
    }

    @After
    public void stop() {
        intake.stop();
    }
}
