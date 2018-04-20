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
 * Created by GCW on 1/6/2018.
 */
@Autonomous(name="Red Parallel Autonomous", group = "Iterative Opmode")
public class RedAutonomous extends LinearOpMode {

    private DcMotor leftBack, leftFront, rightFront, rightBack;
    private Servo flipperLeft, flipperRight, blockerLeft, blockerRight;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        flipperLeft = hardwareMap.servo.get("FL");
        flipperRight = hardwareMap.servo.get("FR");
        blockerLeft = hardwareMap.servo.get("BL");
        blockerRight = hardwareMap.servo.get("BR");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        blockerLeft.setPosition(0);
        blockerRight.setPosition(1);
        MoveF(0.5, 0);
        sleep(1000);
        Stop();
        sleep(500);
        Rotate(-0.6);
        sleep(1200);
        Stop();
        sleep(1000);
        flipperLeft.setPosition(0.8);
        flipperRight.setPosition(0.44);
        sleep(1000);
        flipperLeft.setPosition(0.22);
        flipperRight.setPosition(0.88);
        sleep(500);
        MoveF(0.5, 0);
        sleep(400);
        Stop();
        sleep(500);
        MoveF(-0.4, 0);
        sleep(400);
        Stop();
    }

    private void MoveF (double power, double error) {
        rightFront.setPower(OffsetCalculation.scaled(power + error));
        leftFront.setPower(OffsetCalculation.scaled(power - error));
        rightBack.setPower(OffsetCalculation.scaled(power + error));
        leftBack.setPower(OffsetCalculation.scaled(power - error));
    }

    private void Stop () {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void Rotate (double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }
}