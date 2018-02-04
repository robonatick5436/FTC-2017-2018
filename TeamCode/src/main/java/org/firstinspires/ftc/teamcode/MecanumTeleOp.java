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
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);
        if (gamepad1.a) {
            preDirection = curDirection;
            offset = 0;
        }

        Move();

        telemetry.addData("Offset", offset);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    void Move () {
        float leftY1 = gamepad1.left_stick_y;
        float rightY1 = gamepad1.right_stick_y;
        float z = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower(OffsetCalculation.scaled((leftY1 + z) * slowMultiplier() + offset));
        rightFront.setPower(OffsetCalculation.scaled((rightY1 + z) * slowMultiplier() - offset));
        leftBack.setPower(OffsetCalculation.scaled((leftY1 - z) * slowMultiplier() + offset));
        rightBack.setPower(OffsetCalculation.scaled((rightY1 - z) * slowMultiplier() - offset));
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
}
