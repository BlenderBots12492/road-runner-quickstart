package org.firstinspires.ftc.teamcode.opModes;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTraj", group = "Concept")
public class AutoTraj extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));

    private MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    private DcMotor leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
    private DcMotor rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
    private DcMotor slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");
    private Servo claw = hardwareMap.get(Servo.class, "claw");
    private Servo clawArm = hardwareMap.get(Servo.class, "clawArm");
    public class Intake implements Action {
        @Override
        public boolean slide(double pos) {
            if (leftSlide.getCurrentPosition < distance) {
                while (true) {
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    if (leftSlide.getCurrentPosition() >= distance) {
                        leftSlide.setPower(0);
                        rightSlide.setPower(0);
                        return (true);
                    }
                }
            } else if (leftSlide.getCurrentPosition > distance) {
                while (true) {
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    if (leftSlide.getCurrentPosition() >= distance) {
                        leftSlide.setPower(0);
                        rightSlide.setPower(0);
                        return (true);
                    }
                }
            } else {
                return(true);
            }
        }
        @Override
        public boolean claw(boolean open) {
            if (open) { claw.setPosition(1); }
            else if (!open) { claw.setPosition(0); }
            return(true);
        }
        @Override
        public boolean clawArm(double pos) {
            clawArm.setPosition(pos);
            return(true);
        }
        public boolean rotateSlide(double ang) {
            slideRotator.setTargetPosition(ang*0.0244);
        }
    }
    public Action Intake() {
        return new Intake();
    }
    public void runOpMode() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(38, 38), Math.toRadians(270))
                .splineTo(new Vector2d(60, 61.7), Math.toRadians(45))
                .waitSeconds(1);

        Action Action1 = tab1.build();

        if (isStopRequested()) return;

        Actions.runBlocking(Action1);

    }
}
