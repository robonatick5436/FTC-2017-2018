package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@Autonomous(name="Blue Autonomous", group = "Iterative Opmode")
public class BlueAutonomous extends LinearOpMode {

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB, rackMotor;
    private ColorSensor armColor;
    private Servo arm, harvesterTL, harvesterTR, harvesterBL, harvesterBR;
    private GyroSensor gyro;
    private ModernRoboticsI2cRangeSensor disSensor;
    private float preDirection = 0, curDirection;
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
        rackMotor = hardwareMap.dcMotor.get("RM");

        arm = hardwareMap.servo.get("arm");
        harvesterTL = hardwareMap.servo.get("TL");
        harvesterTR = hardwareMap.servo.get("TR");
        harvesterBL = hardwareMap.servo.get("BL");
        harvesterBR = hardwareMap.servo.get("BR");


        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        harvesterTL.setPosition(0.75);
        harvesterTR.setPosition(0.25);
        harvesterBL.setPosition(0.25);
        harvesterBR.setPosition(0.75);
        sleep(500);
        harvesterTL.setPosition(0.15);
        harvesterTR.setPosition(0.85);
        sleep(500);
        rackMotor.setPower(-0.5);
        sleep(300);

        MoveS(0.9);
        sleep(1500);
        MoveF(0.9, 0);
        sleep(500);
        Stop();
    }

    private void MoveF (double power, double error) {
        rightWheelF.setPower(OffsetCalculation.scaled(power + error));
        leftWheelF.setPower(OffsetCalculation.scaled(power - error));
        rightWheelB.setPower(OffsetCalculation.scaled(-power + error));
        leftWheelB.setPower(OffsetCalculation.scaled(-power - error));
    }

    private void MoveS (double power) {
        leftWheelF.setPower(-power);
        rightWheelF.setPower(power);
        leftWheelB.setPower(power);
        rightWheelB.setPower(-power);
    }

    private void Stop () {
        leftWheelF.setPower(0);
        rightWheelF.setPower(0);
        leftWheelB.setPower(0);
        rightWheelB.setPower(0);
    }

    private void Rotate (double power) {
        leftWheelF.setPower(power);
        rightWheelF.setPower(-power);
        leftWheelB.setPower(power);
        rightWheelB.setPower(-power);
    }

    private void Move (int target, double power, int desiredDirect) {
        int distance = disSensor.rawUltrasonic();
        while (Math.abs(distance - target) > 1) {
            distance = disSensor.rawUltrasonic();
            telemetry.addData("distance", distance);
            telemetry.update();
            double realPower;
            curDirection = gyro.getHeading();
            double offset = OffsetCalculation.offset(curDirection, desiredDirect) * 5;

            if (distance < target * 2) {
                realPower = power / 2;
            } else {
                realPower = power;
            }
            if (distance > target) {
                MoveF(realPower, offset);
            }
            if (distance < target) {
                MoveF(-realPower, offset);
            }
            Stop();
        }
    }

    private void Turn (int target, double power) {
        int zAccumulated = gyro.getHeading();
        if (target > 180) {
            Rotate(0.2);
            sleep(400);
        } else {
            Rotate(-0.2);
            sleep(400);
        }
        while (Math.abs(zAccumulated - target) > 1) {
            zAccumulated = gyro.getHeading();
            telemetry.addData("heading", zAccumulated);
            telemetry.update();
            if (zAccumulated < target) {
                Rotate(power);
            }
            if (zAccumulated > target) {
                Rotate(-power);
            }
        }
        Stop();
    }
}