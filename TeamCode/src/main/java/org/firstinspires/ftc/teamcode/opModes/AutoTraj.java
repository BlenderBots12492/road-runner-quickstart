package org.firstinspires.ftc.teamcode.opModes;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@Autonomous(name = "AutoTraj", group = "Concept")
public class AutoTraj extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    private static Servo clawWrist;
    private static ElapsedTime runtime = new ElapsedTime();
    /*public static void slide(double pos) {
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
    }*/
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
    public void claw(boolean open) {
        if (open) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
    }
    private void reachBasket() {
        sleep(100);
        rotateSlide(300, 1);
        if (isStopRequested()) return;
        slide(1200, 1);
        if (isStopRequested()) return;
        clawArm.setPosition(1);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        claw(true);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        clawArm.setPosition(0);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        rotateSlide(100, 1);
        slide(1300, -1);
        rotateSlide(320, -1);

    }
    private void grabBlock() {
        if (isStopRequested()) return;
        clawArm.setPosition(1);
        sleep(100);
        if (isStopRequested()) return;
        slide(500, 1);
        rotateSlide(350, -1);
        sleep(100);
        claw(false);
        sleep(100);
        slide(500, -1);
        rotateSlide(700, 1);

    }

    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");
        claw = hardwareMap.get(Servo.class, "claw");
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrajectoryBuilder test1;


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.splineTo(new Vector2d(40, 50), Math.toRadians(270))
                //.splineTo(new Vector2d(47, 47), Math.toRadians(45))
                .waitSeconds(1)
                .lineToY(39)
                .turnTo(Math.toRadians(45))
                .lineToY(49);
        Action Action1 = tab1.build();
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToY(38);
                //.turnTo(Math.toRadians(270));
        Action TouchBottom1 = tab3.build();
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .lineToY(0)
                .turnTo(0)
                .lineToX(24);
        Action TouchBottom2 = tab4.build();
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(45)))
                .turnTo(Math.toRadians(260));
                //.lineToYLinearHeading(60,  Math.toRadians(270));
                //.lineToXLinearHeading(48,  Math.toRadians(270));
        Action Toblock1 = tab5.build();
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(45)))
                .turnTo(Math.toRadians(280));
        Action Toblock2 = tab7.build();
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(280)))
                .turnTo(Math.toRadians(45));
        Action TurnBasket1 = tab8.build();

        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(260)))
                .turnTo(Math.toRadians(45));
        Action TurnBasket2 = tab9.build();
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(45)))
                .turnTo(Math.toRadians(270))
                .lineToY(36)
                .turnTo(Math.toRadians(20))
                .lineToY(60);
        Action push = tab6.build();


        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(new Vector2d(47, 47), Math.toRadians(45)))
                .splineTo(new Vector2d(47, 40), Math.toRadians(270))
                .waitSeconds(1);
        Action Action2 = tab2.build();
        waitForStart();
        clawArm.setPosition(0);
        if (isStopRequested()) return;
        Actions.runBlocking(Action1);

        if (isStopRequested()) return;
        sleep(100);
        reachBasket();
        if (isStopRequested()) return;
        Actions.runBlocking(Toblock1);
        grabBlock();
        if (isStopRequested()) return;
        Actions.runBlocking(TurnBasket1);
        if (isStopRequested()) return;
        reachBasket();
        if (isStopRequested()) return;
        Actions.runBlocking(Toblock2);
        if (isStopRequested()) return;
        grabBlock();
        if (isStopRequested()) return;
        Actions.runBlocking(TurnBasket2);
        if (isStopRequested()) return;
        reachBasket();
    }
}