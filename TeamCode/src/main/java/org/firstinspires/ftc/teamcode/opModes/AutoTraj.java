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
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    public static class Intake {
        public static void slide(double pos) {
            if (leftSlide.getCurrentPosition() < pos) {
                while (true) {
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    if (leftSlide.getCurrentPosition() >= pos) {
                        leftSlide.setPower(0);
                        rightSlide.setPower(0);
                        return;
                    }
                }
            } else if (leftSlide.getCurrentPosition() > pos) {
                while (true) {
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    if (leftSlide.getCurrentPosition() >= pos) {
                        leftSlide.setPower(0);
                        rightSlide.setPower(0);
                        return;
                    }
                }
            } else {
            }
        }
        public static void claw(boolean open) {
            if (open) { claw.setPosition(1); }
            else if (!open) { claw.setPosition(0); }
        }
        public static void clawArm(double pos) {
            clawArm.setPosition(pos);
        }
        public static void rotateSlide(double ang) {
            if (ang/0.0244 < slideRotator.getCurrentPosition()) {
                while (true) {
                    slideRotator.setPower(-0.5);
                    if (ang/0.0244 >= slideRotator.getCurrentPosition()) { return; }
                }
            } else if (ang/0.0244 > slideRotator.getCurrentPosition()) {
                while (true) {
                    slideRotator.setPower(0.5);
                    if (ang/0.0244 <= slideRotator.getCurrentPosition()) {
                        return;
                    }
                }
            }
        }
    }
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");
        claw = hardwareMap.get(Servo.class, "claw");
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(38, 38), Math.toRadians(270))
                .splineTo(new Vector2d(47, 47), Math.toRadians(45))
                .waitSeconds(1);

        Action Action1 = tab1.build();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(Action1);
        if (isStopRequested()) return;
        telemetry.addLine().addData("SlideAng", slideRotator.getCurrentPosition());
        telemetry.update();
        Intake.rotateSlide(60);//TODO: DOES NOT WORK!!
        telemetry.addLine().addData("SlideAng", slideRotator.getCurrentPosition());
        telemetry.update();
        if (isStopRequested()) return;
        Intake.slide(1000);
        if (isStopRequested()) return;
        Intake.claw(true);
    }
}
