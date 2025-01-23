package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Auto Clip New", group = "Concept")
public class AutomaticClip extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(-6.75, 61.7, Math.toRadians(270));
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    private static Servo clawWrist;
    private static ElapsedTime runtime = new ElapsedTime();
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
            sleep(10);
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
        int back = 7;
        int forward = 60;
        int sideways = -14;
        int startPush = -45;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(30);
        Action ToBar = tab1.build();
        TrajectoryActionBuilder tab2= drive.actionBuilder(new Pose2d(-6.75, 32, Math.toRadians(270)))
                .lineToY(40)
                .strafeTo(new Vector2d(-35, 40))
                .strafeTo(new Vector2d(-35, back))
                .strafeTo(new Vector2d(startPush, back))
                .strafeTo(new Vector2d(startPush, forward));
        Action ToSampleA = tab2.build();
        TrajectoryActionBuilder tab3= drive.actionBuilder(new Pose2d(-42, 55, Math.toRadians(270)))
                .strafeTo(new Vector2d(startPush, back))
                .strafeTo(new Vector2d(startPush+sideways+2, back))
                .strafeTo(new Vector2d(startPush+sideways+3, forward));
        Action ToSampleB = tab3.build();
        TrajectoryActionBuilder tab4= drive.actionBuilder(new Pose2d(startPush+sideways, forward, Math.toRadians(270)))
                .strafeTo(new Vector2d(startPush+sideways+5, back))
                .strafeTo(new Vector2d(startPush+sideways-3, back))
                .strafeTo(new Vector2d(startPush+sideways-3, forward));
        Action ToSampleC = tab4.build();
        TrajectoryActionBuilder tab5= drive.actionBuilder(new Pose2d(startPush+sideways+3, forward, Math.toRadians(270)))
                .strafeTo(new Vector2d(startPush+sideways+3, 30))
                .waitSeconds(2)
                .strafeTo(new Vector2d(startPush+sideways+3, forward+15));
        Action Wait = tab5.build();






        waitForStart();
        /*clawArm.setPosition(1);
        //sleep(100);
        rotateSlide(465, 1);
        if (isStopRequested()) return;
        slide(490, 1);
        if (isStopRequested()) return;
        Actions.runBlocking(ToBar);
        if (isStopRequested()) return;
        //rotateSlide(50, -1);
        slide(300, -1);
        //clawArm.setPosition(0.25);
        claw(true);
        sleep(1000);
        slide(200, -1);
        rotateSlide(450, -1);*/
        if (isStopRequested()) return;
        Actions.runBlocking(ToSampleA);
        if (isStopRequested()) return;
        Actions.runBlocking(ToSampleB);
        if (isStopRequested()) return;
        Actions.runBlocking(ToSampleC);
        if (isStopRequested()) return;
        Actions.runBlocking(Wait);

    }
}
