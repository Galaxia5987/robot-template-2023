package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import org.junit.After;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestShooter {
    private final Shooter shooter = Shooter.getInstance();

    @Test
    public void setPower() {
        shooter.setPower(1);
    }

    @After
    public void stop() {
        shooter.stop();
    }
}
