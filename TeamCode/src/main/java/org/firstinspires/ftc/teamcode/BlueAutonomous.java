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

    private DcMotor leftWheelF, leftWheelB, rightWheelF, rightWheelB;
    private ColorSensor armColor;
    private Servo arm, harvesterL, harvesterR;
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

        arm = hardwareMap.servo.get("arm");
        harvesterL = hardwareMap.servo.get("HL");
        harvesterR = hardwareMap.servo.get("HR");

//        gyro = hardwareMap.gyroSensor.get("Gyro");
//        disSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ds");
//        disSensor.setI2cAddress(I2cAddr.create8bit(0x90));
        armColor = hardwareMap.colorSensor.get("AC");
        armColor.setI2cAddress(I2cAddr.create8bit(0x4c));
        armColor.enableLed(true);

//        gyro.calibrate();
//        while (gyro.isCalibrating()) {
//            telemetry.addData("Is Calibrating", gyro.isCalibrating());
//            telemetry.update();
//        }
//        gyro.resetZAxisIntegrator();
//        telemetry.addData("Calibration done", "true");
//        telemetry.update();

        rightWheelF.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheelB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        harvesterL.setPosition(0.2);
        harvesterR.setPosition(0.85);
        arm.setPosition(0.35);
        sleep(1000);
        telemetry.addData("Is Red : ", Red());
        telemetry.addData("Red", armColor.red());
        telemetry.addData("Blue", armColor.blue());
        telemetry.update();
        if (armColor.red() > armColor.blue()) {
            MoveF(-0.4, 0);
            sleep(300);
            arm.setPosition(1);
            sleep(400);
            MoveF(-1, 0);
            sleep(600);
        } else {
            MoveF(0.2, 0);
            sleep(300);
            Stop();
            sleep(500);
            arm.setPosition(1);
            sleep(400);
            MoveF(-0.4, 0);
            sleep(2000);
        }
    }

    private void MoveF (double power, double error) {
        rightWheelF.setPower(OffsetCalculation.scaled(power + error));
        leftWheelF.setPower(OffsetCalculation.scaled(power - error));
        rightWheelB.setPower(OffsetCalculation.scaled(-power + error));
        leftWheelB.setPower(OffsetCalculation.scaled(-power - error));
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
