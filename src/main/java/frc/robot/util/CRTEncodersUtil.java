package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class CRTEncodersUtil {
    public static Angle calculateAngle(int pinion1Teeth, int pinion2Teeth, int mainGearTeeth, Angle pinion1Angle, Angle pinion2Angle) {
        double pinion1Ratio = (double)pinion1Teeth/mainGearTeeth;
        double pinion2Ratio = (double)pinion2Teeth/mainGearTeeth;

        double[] pinion1Solutions = createSolutions(createArray(0, pinion2Teeth - 1), pinion1Angle.in(Degrees), pinion1Ratio);
        double[] pinion2Solutions = createSolutions(createArray(0, pinion1Teeth - 1), pinion2Angle.in(Degrees), pinion2Ratio);

        return Degrees.of(findMatch(pinion1Solutions, pinion2Solutions));
    }

    private static int[] createArray(int start, int end) {
        int[] result = new int[end - start + 1];
        for(int i = start; i <= end; i++) {
            result[i - start] = i;
        }

        return result;
    }

    private static double[] createSolutions(int[] values, double pinionAngle, double pinionRatio) {
        double[] solutions = new double[values.length];
        for(int i = 0; i < values.length; i++) {
            solutions[i] = roundToDecimals((values[i] + pinionAngle/360.0)*pinionRatio, 4);
        }

        return solutions;
    }

    private static double findMatch(double[] solutions1, double[] solutions2) {
        for (double solution1 : solutions1) {
            for (double solution2 : solutions2) {
                if (solution1 == solution2) {
                    return solution1;
                }
            }
        }
        return 0.0;
    }

    private static double roundToDecimals(double val, int decimals) {
        val *= Math.pow(10, decimals);
        val += 0.5;
        val = (double)((int)val);
        val /= Math.pow(10, decimals);
        return val;
    }
}
