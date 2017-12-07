package org.firstinspires.ftc.teamcode;

/**
 * Created by GCW on 12/5/2017.
 */

public class OffsetCalculation {
    static double offset (float a, float b) {
        return 1 - Math.cos((a-b) * Math.PI / 180);
    }

    static double desiredAngle (float x, float y) {
        return Math.atan2(x, y);
    }

    static double DegRad (float degree) {
	return degree * Math.PI / 180;
    }

    
}
