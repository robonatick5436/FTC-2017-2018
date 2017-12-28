package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private Servo arm, harvesterL, harvesterR;
    private ColorSensor armColor;
    private GyroSensor gyro;
    private float leftX1,leftY1, rightX1, rightY1, leftY2;
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
        armColor.setI2cAddress(I2cAddr.create8bit(0x4c));
        gyro = hardwareMap.gyroSensor.get("Gyro");

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }

        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        harvesterL.setPosition(0.35);
        harvesterR.setPosition(0.7);
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
        offset = OffsetCalculation.offset(curDirection, preDirection) * 5;

        linearSlide.setPower(leftY2);
        if (gamepad2.right_bumper) {
            harvesterL.setPosition(0.25);
            harvesterR.setPosition(0.85);
        } else {
            harvesterL.setPosition(0.35);
            harvesterR.setPosition(0.70);
        }

        /*linearSlide.setPower(rightY1);
        if (gamepad1.right_bumper) {
            harvesterL.setPosition(0.25);
            harvesterR.setPosition(0.85);
        } else {
            harvesterL.setPosition(0.35);
            harvesterR.setPosition(0.70);
        }*/

        Movement();
        if (gamepad1.left_trigger - gamepad1.right_trigger != 0) {
            ResetDirection();
        }

        telemetry.addData("Offset", offset);
        telemetry.addData("Desired Angle", OffsetCalculation.desiredAngle(leftX1, leftY1));
        telemetry.addData("Red", armColor.red());
        telemetry.addData("Blue", armColor.blue());
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    private void Movement () {
        double z = OffsetCalculation.GamePad1Trigger(gamepad1.left_trigger, gamepad1.right_trigger, gamepad1.left_bumper);
        rightWheelF.setPower(OffsetCalculation.scaled(leftY1 + leftX1 + offset - z));
        leftWheelF.setPower(OffsetCalculation.scaled(leftY1 - leftX1 - offset + z));
        rightWheelB.setPower(OffsetCalculation.scaled(-leftY1 - leftX1 - offset - z));
        leftWheelB.setPower(OffsetCalculation.scaled(-leftY1 + leftX1 + offset + z));
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