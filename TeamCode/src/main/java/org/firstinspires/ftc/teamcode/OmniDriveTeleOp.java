package org.firstinspires.ftc.teamcode;

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

/**
 * Created by GCW on 10/10/2017.
 */

@TeleOp(name="Omni Driving Mode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class OmniDriveTeleOp extends OpMode{

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB, linearSlide;
    private Servo arm, harvesterTL, harvesterTR, harvesterBL, harvesterBR;
    private ColorSensor armColor;
    private GyroSensor gyro;
    private float leftX1,leftY1, rightX1, rightY1, leftY2;
    private float preDirection = 0, curDirection;
    private double offset;
    private boolean flipped;

    private boolean forward () {
        if (leftY1 > 0.9) {
            return true;
        } else {
            return  false;
        }
    }
    private boolean backward () {
        if (leftY1 < -0.9) {
            return true;
        } else {
            return  false;
        }
    }private boolean left () {
        if (leftX1 < -0.9) {
            return true;
        } else {
            return  false;
        }
    }private boolean right () {
        if (leftX1 > 0.9) {
            return true;
        } else {
            return  false;
        }
    }

    @Override
    public void init() {
        leftWheelF = hardwareMap.dcMotor.get("LWF");
        leftWheelB = hardwareMap.dcMotor.get("LWB");
        rightWheelF = hardwareMap.dcMotor.get("RWF");
        rightWheelB = hardwareMap.dcMotor.get("RWB");
        linearSlide = hardwareMap.dcMotor.get("LS");

        harvesterTL = hardwareMap.servo.get("TL");
        harvesterTR = hardwareMap.servo.get("TR");
        arm = hardwareMap.servo.get("arm");

        armColor = hardwareMap.colorSensor.get("AC");
        armColor.setI2cAddress(I2cAddr.create8bit(0x4c));
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }

        armColor.enableLed(true);
        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init_loop() {
        harvesterTL.setPosition(0.35);
        harvesterBL.setPosition(0.35);
        harvesterTR.setPosition(0.7);
        harvesterBR.setPosition(0.7);
        arm.setPosition(1);
    }

    @Override
    public void loop() {
        leftX1 = gamepad1.left_stick_x;
        leftY1 = -gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        rightY1 = -gamepad1.right_stick_y;
        leftY2 = -gamepad2.left_stick_y;

        if (leftY1 == -0) {
            leftY1 = 0;
        }

        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);
        if (gamepad1.a) {
            ResetDirection();
            offset = 0;
        }

        if (gamepad2.a) {
            if (flipped) {
                Flip(450, 0.5);
            } else {
                Flip(-450, 0.5);
            }
        }

        if (gamepad2.right_bumper) {
            harvesterTL.setPosition(0.25);
            harvesterBL.setPosition(0.25);
            harvesterTR.setPosition(0.85);
            harvesterBR.setPosition(0.85);
        } else {
            harvesterTL.setPosition(0.35);
            harvesterBL.setPosition(0.35);
            harvesterTR.setPosition(0.70);
            harvesterBR.setPosition(0.70);
        }

        Movement();

//        telemetry.addData("Offset", offset);
//        telemetry.addData("Desired Angle", OffsetCalculation.desiredAngle(leftX1, leftY1));
//        telemetry.addData("Red", armColor.red());
//        telemetry.addData("Blue", armColor.blue());
//        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    private void Movement () {
        double z = rightX1;

        rightWheelF.setPower(OffsetCalculation.scaled(leftY1 + leftX1 + offset + z));
        leftWheelF.setPower(OffsetCalculation.scaled(leftY1 - leftX1 - offset - z));
        rightWheelB.setPower(OffsetCalculation.scaled(-leftY1 - leftX1 + offset + z));
        leftWheelB.setPower(OffsetCalculation.scaled(-leftY1 + leftX1 - offset - z));
    }

    private void MoveF (double power) {
        leftWheelF.setPower(power);
        leftWheelB.setPower(power);
        rightWheelF.setPower(power);
        rightWheelB.setPower(power);
    }

    private void MoveS (double power) {
        leftWheelF.setPower(-power);
        leftWheelB.setPower(power);
        rightWheelF.setPower(-power);
        rightWheelB.setPower(power);
    }

    private void ResetDirection () {
        preDirection = curDirection;
    }

    private void Flip (int ticks, double power) {
        int targetPos = linearSlide.getCurrentPosition() + ticks;
        linearSlide.setTargetPosition(targetPos);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(power);
        if (!linearSlide.isBusy()) {
            linearSlide.setPower(0);
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}