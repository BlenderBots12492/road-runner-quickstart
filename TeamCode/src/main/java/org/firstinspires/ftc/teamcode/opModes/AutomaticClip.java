package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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
    private Pose2d initialPose = new Pose2d(-4.25, 61.7, Math.toRadians(90));
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    private static Servo clawWrist;
    private static Action ToBar;
    private static ElapsedTime runtime = new ElapsedTime();
    public void slide(int distance) {
        while (opModeIsActive()) {
            if (rightSlide.getCurrentPosition()+10 < distance) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            } else if (rightSlide.getCurrentPosition()-10 > distance) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else {
                leftSlide.setPower(0.06);
                rightSlide.setPower(0.06);
                return;
            }
        }
    }
    public void rotateSlide(double angle) {
        slideRotator.setTargetPosition((int) Math.round(angle/0.0244));
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotator.setPower(1);
    }
    public void rotateSlide(double angle, double power) {
        slideRotator.setTargetPosition((int) Math.round(angle/0.0244));
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotator.setPower(power);
    }
    public boolean isBusy() {
        return Math.abs(slideRotator.getCurrentPosition() - slideRotator.getTargetPosition()) < 50;
    }
    public void waitBusy() {
        while (isBusy() && opModeIsActive()) {
            sleep(100);
        }
    }
    public void claw(boolean open) {
        if (open) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
    }
    public void specimen() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.pose)
                .lineToY(17);
        ToBar = tab1.build();
        rotateSlide(81, 0.7);
        claw(false);
        clawArm.setPosition(0.1); //TODO: adjust value
        clawWrist.setPosition(0.8);
        if (isStopRequested()) return;
        Actions.runBlocking(ToBar);
        if (isStopRequested()) return;
        sleep(100);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.5, 0), 0));
        sleep(100);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(200);
        slide(-800);
        claw(true);
        if (isStopRequested()) return;
        sleep(50);
        if (isStopRequested()) return;
        leftSlide.setPower(-1);
        rightSlide.setPower(-1);
        rotateSlide(25);
        clawArm.setPosition(0.6); //TODO: Adjust Value
        sleep(100);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
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
        int back = 5;
        int forward = 68;
        int sideways = -14;
        int startPush = -50;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(initialPose.position.x, 19));
        ToBar = tab1.build();
        TrajectoryActionBuilder tab2= drive.actionBuilder(new Pose2d(-2.25, 32, Math.toRadians(90)))
                .lineToY(40)
                .strafeTo(new Vector2d(-35, 40))
                .strafeTo(new Vector2d(-35, back))
                .strafeTo(new Vector2d(startPush, back))
                .strafeTo(new Vector2d(startPush, forward));
        Action ToSampleA = tab2.build();
        TrajectoryActionBuilder tab3= drive.actionBuilder(new Pose2d(-42, 55, Math.toRadians(90)))
                .strafeTo(new Vector2d(startPush, back))
                .strafeTo(new Vector2d(startPush+sideways+2, back))
                .strafeTo(new Vector2d(startPush+sideways+3, forward));
        Action ToSampleB = tab3.build();
        TrajectoryActionBuilder tab4= drive.actionBuilder(new Pose2d(startPush+sideways, forward, Math.toRadians(90)))
                .strafeTo(new Vector2d(startPush+sideways+5, back))
                .strafeTo(new Vector2d(startPush+sideways-3, back))
                .strafeTo(new Vector2d(startPush+sideways-3, forward));
        Action ToSampleC = tab4.build();
        TrajectoryActionBuilder tab5= drive.actionBuilder(new Pose2d(startPush+sideways+3, forward, Math.toRadians(90)))
                .strafeTo(new Vector2d(startPush+sideways+3, 30))
                .waitSeconds(2)
                .strafeTo(new Vector2d(startPush+sideways+3, forward+15));
        Action Wait = tab5.build();
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-50, 50, Math.toRadians(90)))
                .lineToY(45)
                .strafeTo(new Vector2d(8, 35))
                .turnTo(90);
        Action NearBar = tab6.build();
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(-50, 70, Math.toRadians(90)))
                .lineToY(45)
                .strafeTo(new Vector2d(0, 35))
                .turnTo(90);
        Action NearBarTwo = tab8.build();
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(0, 20, Math.toRadians(90)))
                .lineToY(45)
                .strafeTo(new Vector2d(-50, 70));
        Action toObserve = tab7.build();

        waitForStart();
        rotateSlide(78, 0.5);
        claw(false);
        clawArm.setPosition(0);
        sleep(600);
        specimen();
        Actions.runBlocking(ToSampleA);
        if (isStopRequested()) return;
        //drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.7, 0), 0));
        //sleep(200);
        //drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        claw(false);
        sleep(300);
        clawArm.setPosition(0);
        Actions.runBlocking(NearBar);
        specimen();
        //Actions.runBlocking(ToSampleC);
        if (isStopRequested()) return;
        //Actions.runBlocking(Wait);
        Actions.runBlocking(toObserve);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.7, 0), 0));
        sleep(200);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        claw(false);
        sleep(200);
        clawArm.setPosition(0);
        Actions.runBlocking(NearBarTwo);
        specimen();
        if (isStopRequested()) return;
        Actions.runBlocking(toObserve);
    }
}
