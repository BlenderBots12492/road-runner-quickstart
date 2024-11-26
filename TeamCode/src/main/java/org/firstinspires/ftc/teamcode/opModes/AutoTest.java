package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AutonomousBasket", group = "Concept")
public class AutoTest extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor rightSlide;
    private DcMotor leftSlide;
    private DcMotor slideRotator;
    private Servo claw;
    private ElapsedTime runtime = new ElapsedTime();
    public void move_left(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.5);
        rightBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        leftBack.setPower(0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(10);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void move_forward(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(10);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void move_forwardSlow(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.2);
        rightBack.setPower(0.2);
        leftFront.setPower(0.2);
        leftBack.setPower(0.2);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(100);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void turn_right(long milliseconds) {
        runtime.reset();
        rightFront.setPower(-0.5);
        rightBack.setPower(-0.5);
        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(100);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void turn_left(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(10);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void move_right(long milliseconds) {
        runtime.reset();
        rightFront.setPower(-0.5);
        rightBack.setPower(0.5);
        leftFront.setPower(0.5);
        leftBack.setPower(-0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(100);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void move_back(long milliseconds) {
        runtime.reset();
        rightFront.setPower(-0.5);
        rightBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(100);
        }
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
    public void slide(int ms, int direction) {
        runtime.reset();
        double power = 0.06;
        if (direction == -1){
            power = 0.0;
        }
        leftSlide.setPower(direction);
        rightSlide.setPower(direction);
        while (opModeIsActive() &&
                runtime.milliseconds() < ms) {
            sleep(100);
        }
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }
    public void rotateSlide(long milliseconds, int direction) {
        runtime.reset();
        slideRotator.setPower(direction);
        while (opModeIsActive() &&
                runtime.milliseconds() < milliseconds) {
            sleep(100);
        }
        slideRotator.setPower(0);
    }
    public void openClaw() {
        claw.setPosition(Range.clip(.3, 0, 1));
    }
    public void runOpMode() {
        waitForStart();
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");

        slideRotator.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        if (opModeIsActive()) {
            move_forward(700);
            sleep(100);
            turn_left(480);
            sleep(100);
            rotateSlide(400, 1);
            sleep(100);
            slide(1400, 1);
            sleep(100);
            move_forwardSlow(2500);
            sleep(100);
            openClaw();
            sleep(100);
            move_back(500);
            sleep(100);
            slide(1400, -1);
            sleep(100);
            rotateSlide(210, -1);

        }
    }
}
