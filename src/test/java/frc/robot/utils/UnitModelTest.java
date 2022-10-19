package frc.robot.utils;

import frc.robot.utils.units.UnitModel;
import org.junit.*;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.io.File;
import java.util.Scanner;

@RunWith(JUnit4.class)
public class UnitModelTest {
    public static final double TICKS_PER_ROTATION = 2048;
    public static final double TICKS_PER_RAD = TICKS_PER_ROTATION / (2 * Math.PI);
    public static final double TICKS_PER_DEGREE = TICKS_PER_ROTATION / 360.0;
    public static final double EPSILON = 1e-4;

    UnitModel[] unitModels = new UnitModel[] {
            new UnitModel(TICKS_PER_ROTATION),
            new UnitModel(TICKS_PER_RAD),
            new UnitModel(TICKS_PER_DEGREE)
    };

    @Test
    public void unitsTest() {
        int numSolutions;
        try {
            Scanner in = new Scanner(new File("test-files/unit-model.txt"));
            for (UnitModel model : unitModels) {
                numSolutions = in.nextInt();
                for (int i = 0; i < numSolutions; i++) {
                    try {
                        Assert.assertEquals(
                                "Unit model " + model.toString() + " to ticks",
                                model.toTicks(in.nextDouble()),
                                in.nextDouble(),
                                EPSILON
                        );
                    } catch (Throwable t) {
                        t.printStackTrace();
                    }
                    try {
                        Assert.assertEquals(
                                "Unit model " + model.toString() + " to units",
                                model.toUnits(in.nextDouble()),
                                in.nextDouble(),
                                EPSILON
                        );
                    } catch (Throwable t) {
                        t.printStackTrace();
                    }
                    try {
                        Assert.assertEquals(
                                "Unit model " + model.toString() + " to ticks100ms",
                                model.toTicks100ms(in.nextDouble()),
                                in.nextDouble(),
                                EPSILON
                        );
                    } catch (Throwable t) {
                        t.printStackTrace();
                    }
                    try {
                        Assert.assertEquals(
                                "Unit model " + model.toString() + " to velocity",
                                model.toVelocity(in.nextDouble()),
                                in.nextDouble(),
                                EPSILON
                        );
                    } catch (Throwable t) {
                        t.printStackTrace();
                    }
                }
            }
        } catch (Throwable t) {
            t.printStackTrace();
        }

    }
}
