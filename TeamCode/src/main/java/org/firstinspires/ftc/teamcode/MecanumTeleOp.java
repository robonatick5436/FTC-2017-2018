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

    DcMotor leftFront, leftBack, rightFront, rightBack, harvesterLeft, harvesterRight, liftLeft, liftRight;
    Servo flipperUp, flipperDown, blockerLeft, blockerRight;
    GyroSensor gyro;

    private float preDirection = 0, curDirection;
    private double offset;

    private Pid leftDrive = null;
    private Pid rightDrive = null;

    private final double ticksPerRevolution = 1120;  // Get for your motor and gearing.
    private double prevTime;  // The last time loop() was called.
    private int prevLeftEncoderPosition;   // Encoder tick at last call to loop().
    private int prevRightEncoderPosition;  // Encoder tick at last call to loop().
    private final double drivePidKp = 0.8;     // Tuning variable for PID.
    private final double drivePidTi = 1;   // Eliminate integral error in 1 sec.
    private final double drivePidTd = 0.1;   // Account for error in 0.1 sec.
    // Protect against integral windup by limiting integral term.
    private final double drivePidIntMax = 0.3;  // Limit to max speed.
    private final double driveOutMax = 1.0;  // Motor output limited to 100%.

    float slowMultiplier () {
        if (gamepad1.right_bumper) {
            return 1;
        } else {
            return 0.6f;
        }
    }

    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        harvesterLeft = hardwareMap.dcMotor.get("HL");
        harvesterRight = hardwareMap.dcMotor.get("HR");
        liftLeft = hardwareMap.dcMotor.get("LL");
        liftRight = hardwareMap.dcMotor.get("LR");
        flipperUp = hardwareMap.servo.get("Cice");
        flipperDown = hardwareMap.servo.get("FD");
        blockerLeft = hardwareMap.servo.get("BL");
        blockerRight = hardwareMap.servo.get("BR");
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        harvesterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
//        // Compute speed of left,right motors.
//        double deltaTime = time - prevTime;
//        double leftSpeed = (leftFront.getCurrentPosition() - prevLeftEncoderPosition) /
//                deltaTime;
//        double rightSpeed = (rightFront.getCurrentPosition() - prevRightEncoderPosition) /
//                deltaTime;
//        // Track last loop() values.
//        prevTime = time;
//        prevLeftEncoderPosition = leftFront.getCurrentPosition();
//        prevRightEncoderPosition = rightFront.getCurrentPosition();

        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);
        if (gamepad1.a || gamepad1.left_stick_y + gamepad1.right_stick_y != 0) {
            preDirection = curDirection;
            offset = 0;
        }

//        // Use Pid to compute motor powers to achieve wheel velocity.
//        left = leftDrive.update(gamepad1.left_stick_y, leftSpeed, deltaTime);
//        right = rightDrive.update(gamepad1.right_stick_y, rightSpeed, deltaTime);
//        // Clamp motor powers.
//        Vector2d motorPower = new Vector2d(left, right);
//        clampPowers(motorPower);
//        left = motorPower.getX();
//        right = motorPower.getY();

        harvesterLeft.setPower(gamepad2.left_stick_y * 0.5);
        harvesterRight.setPower(gamepad2.right_stick_y * 0.5);

        if (gamepad2.dpad_up) {
            liftLeft.setPower(1);
            liftRight.setPower(1);
        } else if (gamepad2.dpad_down) {
            liftLeft.setPower(-1);
            liftRight.setPower(-1);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }

        if (gamepad2.a) {
            blockerLeft.setPosition(1);
            blockerRight.setPosition(0);
            flipperUp.setPosition(0.2);
            flipperDown.setPosition(0.8);
        }
        if (gamepad2.b) {
            blockerLeft.setPosition(0.7);
            blockerRight.setPosition(0.3);
            flipperUp.setPosition(0.8);
            flipperDown.setPosition(0.2);
        }

        Move(left, right);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    void Move (double l, double r) {
        float z = gamepad1.left_trigger - gamepad1.right_trigger;

//        leftFront.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier() + offset));
//        rightFront.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier() - offset));
//        leftBack.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier() + offset));
//        rightBack.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier() - offset));
        leftFront.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier()));
        rightFront.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier()));
        leftBack.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier()));
        rightBack.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier()));
        telemetry.addData("Left Power", l);
        telemetry.addData("Right Power", r);
        telemetry.addData("offset", offset);
        telemetry.update();
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
