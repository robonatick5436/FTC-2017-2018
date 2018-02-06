package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;

//import com.github.pmtischler.control.Mecanum;


/**
 * Created by GCW on 2/4/2018.
 */

@TeleOp(name="Mecanum Driving Mode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class MecanumTeleOp extends OpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;
    GyroSensor gyro;

    private float preDirection = 0, curDirection;
    private double offset;

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    private final double ticksPerRevolution = 1000;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLeftEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRightEncoderPosition;  // Encoder tick at last call to loop().
    private final double drivePidKp = 1;     // Tuning variable for PID.
    private final double drivePidTi = 1.0;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 1.0;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

    float slowMultiplier () {
        if (gamepad1.right_bumper) {
            return 1;
        } else {
            return 0.4f;
        }
    }

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ... other code

        leftDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
        rightDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        // Compute speed of left,right motors.
        double deltaTime = time - prevTime;
        double leftSpeed = (leftFront.getCurrentPosition() - prevLeftEncoderPosition) /
                deltaTime;
        double rightSpeed = (rightFront.getCurrentPosition() - prevRightEncoderPosition) /
                deltaTime;
        // Track last loop() values.
        prevTime = time;
        prevLeftEncoderPosition = leftFront.getCurrentPosition();
        prevRightEncoderPosition = rightFront.getCurrentPosition();

        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);
        if (gamepad1.a || gamepad1.left_stick_y + gamepad1.right_stick_y != 0) {
            preDirection = curDirection;
            offset = 0;
        }

        // Use Pid to compute motor powers to achieve wheel velocity.
        left = leftDrive.update(1, leftSpeed, deltaTime);
        right = rightDrive.update(1, rightSpeed, deltaTime);
        // Clamp motor powers.
        Vector2d motorPower = new Vector2d(left, right);
        clampPowers(motorPower);
        left = motorPower.getX();
        right = motorPower.getY();

        Move(left, right);

        telemetry.addData("Offset", offset);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    void Move (double l, double r) {
        float z = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier() + offset));
        rightFront.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier() - offset));
        leftBack.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier() + offset));
        rightBack.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier() - offset));
    }

    void BetterMove () {
        // Convert joysticks to desired motion.
        Mecanum.Motion motion = Mecanum.joystickToMotion(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Convert desired motion to wheel powers, with power clamping.
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        leftFront.setPower(wheels.frontLeft);
        rightFront.setPower(wheels.frontRight);
        leftBack.setPower(wheels.backLeft);
        rightBack.setPower(wheels.backRight);
    }

    void clampPowers (Vector2d v2d) {
        Pid.clampValue(v2d.getX(), -1, 1);
        Pid.clampValue(v2d.getY(), -1, 1);
    }
}
