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

@TeleOp(name="Simple Driving Mode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class SimpleTeleOp extends OpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack, harvesterLeft, harvesterRight;
    GyroSensor gyro;

    private float preDirection = 0, curDirection;
    private double offset;
    private double leftPower, rightPower;

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
        harvesterLeft = hardwareMap.dcMotor.get("HL");
        harvesterRight = hardwareMap.dcMotor.get("HR");
        gyro = hardwareMap.gyroSensor.get("Gyro");

//        gyro.resetZAxisIntegrator();
//        gyro.calibrate();
//        while (gyro.isCalibrating()) {
//            telemetry.addData("Is Calibrating", gyro.isCalibrating());
//            telemetry.update();
//        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        harvesterRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        harvesterLeft.setPower(gamepad2.left_stick_y);
        harvesterRight.setPower(gamepad2.right_stick_y);

        Move(left, right);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    void Move (double l, double r) {
//        float z = gamepad1.left_trigger - gamepad1.right_trigger;
//
//        leftFront.setPower(OffsetCalculation.scaled((l + z) * slowMultiplier() + offset));
//        rightFront.setPower(OffsetCalculation.scaled((r + z) * slowMultiplier() - offset));
//        leftBack.setPower(OffsetCalculation.scaled((l - z) * slowMultiplier() + offset));
//        rightBack.setPower(OffsetCalculation.scaled((r - z) * slowMultiplier() - offset));
        leftFront.setPower(OffsetCalculation.scaled(l));
        leftBack.setPower(OffsetCalculation.scaled(l));
        rightFront.setPower(OffsetCalculation.scaled(r));
        rightBack.setPower(OffsetCalculation.scaled(r));
//        telemetry.addData("Left Power", l);
//        telemetry.addData("Right Power", r);
//        telemetry.update();
    }
}
