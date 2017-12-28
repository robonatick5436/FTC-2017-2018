package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by GCW on 12/14/2017.
 */
@Autonomous(name="Test Autonomous", group = "Iterative Opmode")
public class TestAutonomous extends LinearOpMode {

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB;
    private ColorSensor armColor;
    private Servo arm, harvesterL, harvesterR;
    private GyroSensor gyro;
    private ModernRoboticsI2cRangeSensor disSensor;
    private float preDirection = 0, curDirection;
    private float dis;
    private double offset;
    private double rotationPower;

    private boolean Red () {
        if (armColor.red() > armColor.blue()) {
            return true;
        } else {
            return  false;
        }
    }

    @Override
    public void runOpMode() {
        leftWheelF = hardwareMap.dcMotor.get("LWF");
        leftWheelB = hardwareMap.dcMotor.get("LWB");
        rightWheelF = hardwareMap.dcMotor.get("RWF");
        rightWheelB = hardwareMap.dcMotor.get("RWB");

        arm = hardwareMap.servo.get("arm");
        harvesterL = hardwareMap.servo.get("HL");
        harvesterR = hardwareMap.servo.get("HR");

        gyro = hardwareMap.gyroSensor.get("Gyro");
        disSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ds");
        armColor = hardwareMap.colorSensor.get("AC");
        armColor.setI2cAddress(I2cAddr.create8bit(0x4c));

        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            telemetry.addData("Is Calibrating", gyro.isCalibrating());
            telemetry.update();
        }
        if (!gyro.isCalibrating()) {
            telemetry.addData("Calibration done", "true");
            telemetry.update();
        }

        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        Turn(45, 0.5);

//        while (opModeIsActive()) {
//            curDirection = gyro.getHeading();
//            offset = OffsetCalculation.offset(curDirection, preDirection) * 5;
//            telemetry.addData("Offset", offset);
//            telemetry.addData("Heading", curDirection);
//            telemetry.update();
//        }
//
//        harvesterL.setPosition(0.2);
//        harvesterR.setPosition(0.85);
//        arm.setPosition(0.35);
//        sleep(1000);
//        telemetry.addData("Is Red : ", Red());
//        telemetry.update();
//        if (Red()) {
//            MoveF(-0.4);
//            sleep(300);
//            arm.setPosition(1);
//            sleep(400);
//            MoveF(-1);
//            sleep(600);
//        } else {
//            MoveF(0.2);
//            sleep(300);
//            MoveF(0);
//            sleep(500);
//            arm.setPosition(1);
//            sleep(400);
//            MoveF(-0.4);
//            sleep(2000);
//        }
    }

    private void MoveF (double power) {
        rightWheelF.setPower(OffsetCalculation.scaled(power + offset));
        leftWheelF.setPower(OffsetCalculation.scaled(power - offset));
        rightWheelB.setPower(OffsetCalculation.scaled(-power - offset));
        leftWheelB.setPower(OffsetCalculation.scaled(-power + offset));
    }

    private void Rotate (double power) {
        leftWheelF.setPower(power);
        rightWheelF.setPower(-power);
        leftWheelB.setPower(power);
        rightWheelB.setPower(-power);
    }

    private void Turn (int target, double power) {
        int zAccumulated = 0;
        telemetry.addData("first stage", "true");
        telemetry.update();
        while (Math.abs(zAccumulated - target) > 1) {
            telemetry.addData("second stage", "true");
            telemetry.update();
            zAccumulated = gyro.getHeading();
            double realPower = OffsetCalculation.scaled(power * (zAccumulated - target) / 10);
            if (zAccumulated < target) {
                telemetry.addData("third stage", "true");
                telemetry.update();
                leftWheelF.setPower(realPower);
                rightWheelF.setPower(-realPower);
                leftWheelB.setPower(realPower);
                rightWheelB.setPower(-realPower);
            }
            if (zAccumulated > target) {
                telemetry.addData("third stage", "true");
                telemetry.update();
                leftWheelF.setPower(-realPower);
                rightWheelF.setPower(realPower);
                leftWheelB.setPower(-realPower);
                rightWheelB.setPower(realPower);
            }
        }

        leftWheelF.setPower(0);
        rightWheelF.setPower(0);
        leftWheelB.setPower(0);
        rightWheelB.setPower(0);
    }
}
