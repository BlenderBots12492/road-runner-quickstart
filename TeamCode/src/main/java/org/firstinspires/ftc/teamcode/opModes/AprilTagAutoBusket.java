package org.firstinspires.ftc.teamcode.opModes;
import android.support.v4.app.INotificationSideChannel;
import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
@Autonomous(name = "ImprovedAutoPlasticContainer", group = "Concept")
public class AprilTagAutoBusket extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    private static Servo clawWrist;
    private static ElapsedTime runtime = new ElapsedTime();
    private static int side = 1;
    private static int sideAng = 0;
    private static AprilTagProcessor tagProcessor;
    public void slide(double distance) {
        while (opModeIsActive()) {
            if ((-leftSlide.getCurrentPosition())+10 < distance) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else if (leftSlide.getCurrentPosition()-10 > distance) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            } else {
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                return;
            }
        }
    }
    public void rotateSlide(double angle) {
        double distance = angle/0.0244;
        while (opModeIsActive()) {
            if (slideRotator.getCurrentPosition()+10 < distance) {
                slideRotator.setPower(1);
            } else if (slideRotator.getCurrentPosition()-10 > distance) {
                slideRotator.setPower(-1);
            } else {
                slideRotator.setPower(0);
                return;
            }
        }
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
        rotateSlide(60);
        if (isStopRequested()) return;
        slide(2500);
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
        rotateSlide(65);
        sleep(1000);
        slide(0);
        sleep(1000);
        rotateSlide(45);
    }
    private void grabBlock() {
        if (isStopRequested()) return;
        clawArm.setPosition(1);
        sleep(100);
        if (isStopRequested()) return;
        slide(1300);
        rotateSlide(0);
        sleep(150);
        claw(false);
        sleep(100);
        slide(0);
        rotateSlide(45);
        sleep(250);

    }
    private Pose2d getPose() {
        if (tagProcessor.getDetections().size() > 0) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            Position pose = tag.robotPose.getPosition();
            if (pose.y < 0) {
                side = -1;
                sideAng = -180;
            } else {
                sideAng = 0;
                side = 1;
            }
            drive.pose = new Pose2d(pose.x, pose.y, tag.robotPose.getOrientation().getYaw());
            return(new Pose2d(pose.x, pose.y, tag.robotPose.getOrientation().getYaw()));
        }
        return(drive.pose);
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
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Position cameraPosition = new Position(DistanceUnit.INCH, -7, 8, 1, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 90, 0);

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes (true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        clawArm.setPosition(0);
        if (isStopRequested()) return;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(getPose())
                .waitSeconds(1)
                .lineToY(39*side)
                .turnTo(Math.toRadians(45+sideAng))
                .lineToY(49*side);
        Action Action1 = tab1.build();
        Actions.runBlocking(Action1);

        if (isStopRequested()) return;
        sleep(100);
        reachBasket();
        if (isStopRequested()) return;
        TrajectoryActionBuilder tab5 = drive.actionBuilder(getPose())
                .turnTo(Math.toRadians(264+sideAng));
        Action Toblock1 = tab5.build();
        Actions.runBlocking(Toblock1);
        grabBlock();
        if (isStopRequested()) return;
        TrajectoryActionBuilder tab8 = drive.actionBuilder(getPose())
                .turnTo(Math.toRadians(45+sideAng));
        Action TurnBasket1 = tab8.build();
        Actions.runBlocking(TurnBasket1);
        if (isStopRequested()) return;
        reachBasket();
        if (isStopRequested()) return;
        TrajectoryActionBuilder tab7 = drive.actionBuilder(getPose())
                .turnTo(Math.toRadians(280+sideAng));
        Action Toblock2 = tab7.build();
        Actions.runBlocking(Toblock2);
        if (isStopRequested()) return;
        grabBlock();
        if (isStopRequested()) return;
        TrajectoryActionBuilder tab9 = drive.actionBuilder(getPose())
                .turnTo(Math.toRadians(45+sideAng));
        Action TurnBasket2 = tab9.build();
        Actions.runBlocking(TurnBasket2);
        if (isStopRequested()) return;
        reachBasket();
    }
}

/*while (adrianIsAlive  == true) {
     worldHunger = notSolved;
        }*/