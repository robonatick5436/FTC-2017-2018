package org.firstinspires.ftc.teamcode;

/**
 * Created by Teacher on 2/6/2018.
 */

public class FeedbackLinearizer {
    /**
     * Creates a FeedbackLinearizer with robot and control paramters.
     * @param wheelRadius The wheel radius; R in the equations.
     * @param wheelBaseline The distance between the wheels; L in the equations.
     * @param feedbackEpsilon The feedback algo epsilon; ε in the equations.
     */
    public FeedbackLinearizer(double wheelRadius, double wheelBaseline,
                              double feedbackEpsilon) {
        this.wheelRadius = wheelRadius;
        this.wheelBaseline = wheelBaseline;
        this.feedbackEpsilon = feedbackEpsilon;
    }

    /**
     * Gets wheel velocities to match a desired robot velocity.
     * @param velocity The desired velocity of the robot in robot frame.
     * @return The left wheel (x) and right wheel (y) angular velocities.
     */
    public Vector2d getWheelVelocitiesForRobotVelocity(Vector2d velocity) {
        // Convert robot velocity into forward and angular components.
        Vector2d forwardAndAngularVel = feedbackLinearize(velocity);
        // Convert forward and angular to wheel velocities.
        return convertToWheelVelocities(forwardAndAngularVel.getX(),
                forwardAndAngularVel.getY());
    }

    /**
     * Convert a velocity in robot frame to forward and angular velocities.
     * @param velocity The desired velocity of the robot in robot frame.
     * @return The forward (x) and angular (y) velocities (radians/sec).
     */
    public Vector2d feedbackLinearize(Vector2d velocity) {
        return new Vector2d(velocity.getX(),
                velocity.getY() / feedbackEpsilon);
    }

    /**
     * Convert a forward and angular velocities to wheel velocities.
     * @param forwardVelocity The forward velocity.
     * @param angularVelocity The angular velocity (radians/sec).
     * @return The left wheel (x) and right wheel (y) angular velocities.
     */
    public Vector2d convertToWheelVelocities(double forwardVelocity,
                                             double angularVelocity) {
        double vfOverR = forwardVelocity / wheelRadius;
        double wLOver2R = angularVelocity * wheelBaseline / (2 * wheelRadius);
        return new Vector2d(vfOverR - wLOver2R, vfOverR + wLOver2R);
    }

    // The wheel radius; R in the equations.
    private double wheelRadius;
    // The distance between the wheels; L in the equations.
    private double wheelBaseline;
    // The feedback algo epsilon; ε in the equations.
    private double feedbackEpsilon;
}
