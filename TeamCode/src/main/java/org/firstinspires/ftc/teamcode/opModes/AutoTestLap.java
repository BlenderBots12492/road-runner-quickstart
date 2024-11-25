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
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous(name = "AutoTestLap", group = "Concept")
public class AutoTestLap extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private ElapsedTime runtime = new ElapsedTime();
    public void move_left(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.5);
        rightBack.setPower(-0.5);
        leftFront.setPower(-0.5);
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
    public void move_forward(long milliseconds) {
        runtime.reset();
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
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
            sleep(100);
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
    public void runOpMode() {
        waitForStart();
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        if (opModeIsActive()) {
            while(opModeIsActive()) {
                move_forward(3200);
                sleep(100);
                turn_right(870);
            }
        }
    }
}

