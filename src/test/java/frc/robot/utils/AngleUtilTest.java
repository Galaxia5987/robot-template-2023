package frc.robot.utils;

import frc.robot.utils.math.AngleUtil;
import org.junit.*;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Scanner;

@RunWith(JUnit4.class)
public class AngleUtilTest {
    public static final double EPSILON = 1e-4;

    @Test
    public void testAngleUtil() {
        try {
            Scanner in = new Scanner(new File("test-files/angle-util.txt"));
            int numAbsoluteTests = in.nextInt();
            for (int i = 0; i < numAbsoluteTests; i++) {
                try {
                    AngleUtil.Angle angle = inputAngle(in);
                    Assert.assertEquals(
                            "Absolute test - " + angle.toString(),
                            in.nextDouble(),
                            angle.changeCoordinateSystem(AngleUtil.CoordinateSystem.ABSOLUTE).value,
                            EPSILON
                    );
                } catch (Throwable t) {
                    t.printStackTrace();
                }
            }

            int numDifferenceTests = in.nextInt();
            for (int i = 0; i < numDifferenceTests; i++) {
                try {
                    AngleUtil.Angle from = inputAngle(in);
                    AngleUtil.Angle to = inputAngle(in);
                    Assert.assertEquals(
                            "Difference test\n" + from.toString() + "\n" + to.toString() + "\n",
                            in.nextDouble(),
                            from.minus(to),
                            EPSILON
                    );
                } catch (Throwable t) {
                    t.printStackTrace();
                }
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }
    }

    public AngleUtil.Angle inputAngle(Scanner in) {
        boolean invertX = in.nextInt() != 0,
                invertY = in.nextInt() != 0,
                clockwise = in.nextInt() == 0;
        double angle = in.nextDouble();
        return new AngleUtil.Angle(AngleUtil.CoordinateSystem.of(invertX, invertY, clockwise), angle);
    }
}