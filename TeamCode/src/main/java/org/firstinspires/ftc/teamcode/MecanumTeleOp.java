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
    Servo flipperLeft, flipperRight, blockerLeft, blockerRight;
    GyroSensor gyro;

    private static int ticksPerRev = 1120;

    private float preDirection = 0, curDirection;
    private float position;
    private double offset;
    private boolean flipped;
    private boolean is;

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
        if (gamepad1.y) {
            return 1;
        } else if (gamepad1.right_bumper) {
            return 0.27f;
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
        harvesterLeft = hardwareMap.dcMotor.get("HL");
        harvesterRight = hardwareMap.dcMotor.get("HR");
        liftLeft = hardwareMap.dcMotor.get("LL");
        liftRight = hardwareMap.dcMotor.get("LR");
        flipperLeft = hardwareMap.servo.get("FL");
        flipperRight = hardwareMap.servo.get("FR");
        blockerLeft = hardwareMap.servo.get("BL");
        blockerRight = hardwareMap.servo.get("BR");
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        harvesterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
        rightDrive = new Pid(drivePidKp, drivePidTi, drivePidTd,
                -drivePidIntMax, drivePidIntMax,
                -driveOutMax, driveOutMax);
    }

    @Override
    public void init_loop() {
        flipperLeft.setPosition(0.22);
        flipperRight.setPosition(0.88);
        blockerLeft.setPosition(0);
        blockerRight.setPosition(1);
    }

    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);

        if (gamepad1.left_bumper) {
            preDirection = curDirection;
            offset = 0;
        }

        if (!flipped) {
            harvesterLeft.setPower(gamepad2.left_stick_y * 0.5);
            harvesterRight.setPower(gamepad2.right_stick_y * 0.5);
        } else {
            MoveSlide();
        }

        if (gamepad2.a) {
            flipped = true;
            flipperLeft.setPosition(0.8);
            flipperRight.setPosition(0.44);
        }
        if (gamepad2.x) {
            flipped = true;
            flipperLeft.setPosition(0.5);
            flipperRight.setPosition(0.67);
        }
        if (gamepad2.y) {
            flipped = false;
            flipperLeft.setPosition(0.22);
            flipperRight.setPosition(0.88);
        }
        if (gamepad2.left_bumper) {
            blockerLeft.setPosition(0);
            blockerRight.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            blockerLeft.setPosition(0.2);
            blockerRight.setPosition(0.8);
        }


        Move(left, right);

        telemetry.addData("L", leftBack.getPower());
        telemetry.addData("R", rightBack.getPower());
        telemetry.addData("offset", offset);
        telemetry.addData("L Pos", leftBack.getCurrentPosition());
        telemetry.addData("R Pos", rightBack.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    void Move (double l, double r) {
        float z = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier() + offset));
        rightFront.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier() - offset));
        leftBack.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier() + offset));
        rightBack.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier() - offset));
//        leftFront.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier()));
//        rightFront.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier()));
//        leftBack.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier()));
//        rightBack.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier()));
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

    void MoveSlide () {
        if (gamepad2.dpad_up) {
            liftLeft.setPower(1);
            liftRight.setPower(1);
        } else if (gamepad2.dpad_down) {
            liftLeft.setPower(-1);
            liftRight.setPower(-1);
        } else if (gamepad2.dpad_left) {
            liftLeft.setPower(-1);
        } else if (gamepad2.dpad_right) {
            liftRight.setPower(-1);
        } else {
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }
    }
}
