package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by GCW on 12/5/2017.
 */

public class OffsetCalculation {
    static double offset (float a, float b) {
        double d = 1 - Math.cos((a-b) * Math.PI / 180);
        if (d > 0.08 || d < -0.08) {
            return d;
        } else {
            return 0;
        }
    }

    static double desiredAngle (float x, float y) {
        return Math.atan2(x, y);
    }

    static double DegRad (float degree) {
	return degree * Math.PI / 180;
    }

    static double GamePad1Trigger (float left, float right, boolean fast) {
        if (fast) {
            return left - right;
        } else {
            return (left - right) / 8;
        }
    }

    static double scaled (double input) { return Range.clip(input, -1, 1); }
}
