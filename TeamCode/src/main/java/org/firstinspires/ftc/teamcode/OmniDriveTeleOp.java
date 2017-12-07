package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by GCW on 10/10/2017.
 */

@TeleOp(name="Omni Driving Mode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class OmniDriveTeleOp extends OpMode{

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB, linearSlide;
    private Servo arm, harvesterL, harvesterR;
    private ColorSensor armColor;
    private GyroSensor gyro;
    private float leftX1,leftY1, rightX1, leftY2;
    private float preDirection = 0, curDirection;
    private double offset;

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

        harvesterL = hardwareMap.servo.get("HL");
        harvesterR = hardwareMap.servo.get("HR");
        arm = hardwareMap.servo.get("arm");

        armColor = hardwareMap.colorSensor.get("AC");
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
            //Thread.yield();
        }

        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        harvesterL.setPosition(0.2);
        harvesterR.setPosition(0.85);
        arm.setPosition(1);
    }

    @Override
    public void loop() {
        leftX1 = gamepad1.left_stick_x;
        leftY1 = -gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        leftY2 = -gamepad2.left_stick_y;

        //if (leftY1 == -0) {
        //    leftY1 = 0;
        //}

        curDirection = gyro.getHeading();
        offset = OffsetCalculation.offset(curDirection, preDirection);

        linearSlide.setPower(leftY2);
        if (gamepad2.right_bumper) {
            harvesterR.setPosition(1);
            harvesterL.setPosition(0.1);
        } else {
            harvesterR.setPosition(0.9);
            harvesterL.setPosition(0.2);
        }

        Movement();
        //MoveF(leftY1);

        telemetry.addData("Offset", offset);
        telemetry.addData("Desired Angle", OffsetCalculation.desiredAngle(leftX1, leftY1));
        telemetry.addData("Current Heading", curDirection);
        telemetry.addData("Previous Heading", preDirection);
        telemetry.addData("Y", leftY1);
        telemetry.addData("X", leftX1);
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    private void Movement () {
        double z = OffsetCalculation.GamePad1Trigger(gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.right_bumper);
        rightWheelF.setPower(OffsetCalculation.scaled(leftY1 + leftX1 - z));
        leftWheelF.setPower(OffsetCalculation.scaled(leftY1 - leftX1 + z));
        rightWheelB.setPower(OffsetCalculation.scaled(-leftY1 - leftX1 - z));
        leftWheelB.setPower(OffsetCalculation.scaled(-leftY1 + leftX1 + z));
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

    private void Rotate () {
        double power = gamepad1.right_trigger - gamepad1.left_trigger;
        leftWheelF.setPower(power);
        leftWheelB.setPower(power);
        rightWheelF.setPower(-power);
        rightWheelB.setPower(-power);
        preDirection = gyro.getHeading();
    }

    private void Restriction () {
        if (forward() || backward()) {
            MoveF(leftY1);
        }else if (left() || right()) {
            MoveS(leftX1);
        } else {
            Movement();
        }
    }
}