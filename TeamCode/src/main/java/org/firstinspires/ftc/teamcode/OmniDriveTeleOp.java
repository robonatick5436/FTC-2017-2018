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

/**
 * Created by GCW on 10/10/2017.
 */

@TeleOp(name="Omni Driving Mode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
public class OmniDriveTeleOp extends OpMode{

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB, linearSlide;
    private Servo arm;
    private CRServo harvesterL, harvesterR;
    private ColorSensor armColor;
    private GyroSensor gyro;
    private float leftX1,leftY1, rightX1, leftY2;

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

        harvesterL = hardwareMap.crservo.get("HL");
        harvesterR = hardwareMap.crservo.get("HR");
        arm = hardwareMap.servo.get("arm");

        armColor = hardwareMap.colorSensor.get("AC");
        gyro = hardwareMap.gyroSensor.get("Gyro");

        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);
        //harvesterR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void loop() {
        leftX1 = gamepad1.left_stick_x;
        leftY1 = gamepad1.left_stick_y;
        rightX1 = gamepad1.right_stick_x;
        leftY2 = gamepad2.left_stick_y;

        linearSlide.setPower(leftY2);
        harvesterL.setPower((gamepad1.left_trigger - gamepad2.right_trigger) * time);
        harvesterR.setPower((gamepad1.left_trigger - gamepad2.right_trigger) * time);

        if (rightX1 == 0) {
            Restriction();
        } else {
            Rotate();
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }

    private void Movement () {
        leftWheelF.setPower(leftY1 + leftX1);
        leftWheelB.setPower(leftY1 - leftX1);
        rightWheelF.setPower(leftY1 + leftX1);
        rightWheelB.setPower(leftY1 - leftX1);
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
        leftWheelF.setPower(rightX1);
        leftWheelB.setPower(rightX1);
        rightWheelF.setPower(-rightX1);
        rightWheelB.setPower(-rightX1);
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