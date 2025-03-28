package org.firstinspires.ftc.teamcode.opModes;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Size;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;
import java.util.List;

@Autonomous(name = "AutoBusket", group = "Concept")
public class AutoBucket extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));
    private MecanumDrive drive;
    private static DcMotor leftSlide;
    private static DcMotor rightSlide;
    private static DcMotor slideRotator;
    private static Servo claw;
    private static Servo clawArm;
    private static Servo clawWrist;
    private static ElapsedTime runtime = new ElapsedTime();
    private Point MoveTo;
    private ColorBlobLocatorProcessor colorLocator;
    public void slide(int distance) {
        while (opModeIsActive()) {
            telemetry.addLine().addData("SlidePos", leftSlide.getCurrentPosition());
            telemetry.update();
            if (leftSlide.getCurrentPosition()+10 < distance) {
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
            } else if (leftSlide.getCurrentPosition()-10 > distance) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            } else {
                leftSlide.setPower(0.06);
                rightSlide.setPower(0.06);
                return;
            }
        }
    }
    public void rotateSlidePrim(double angle) {
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
    public void rotateSlide(double angle) {
        slideRotator.setTargetPosition((int) Math.round(angle/0.0244));
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotator.setPower(1);//65
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
        slide(-2700);
        clawArm.setPosition(1);
        sleep(500);
        claw(true);
        sleep(1000);
        clawArm.setPosition(0);
        sleep(500);
        rotateSlide(65);
        sleep(500);
        leftSlide.setPower(-1);
        rightSlide.setPower(-1);
        sleep(100);
        rotateSlide(45);
        sleep(1000);
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
    private void bend() {
        claw(true);
        clawArm.setPosition(1);
        sleep(300);
        rotateSlide(0);
        if (slideRotator.getCurrentPosition() < 20) {
            claw(false);
            sleep(100);
            clawArm.setPosition(0);
            rotateSlide(45);
        }
    }
    private void grabBlock() {
        double ForV;
        double AngV;
        int pxX = 160;
        int pxY = 210;
        int angS = 200;
        int forS = 280;
        rotateSlide(30);
        while (opModeIsActive() || opModeInInit())
        {

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();


            ColorBlobLocatorProcessor.Util.filterByArea(100, 20000, blobs);  // filter out very small blobs.

            // Display the size (area) and center location for each Blob.
            if (!blobs.isEmpty()) {
                for (ColorBlobLocatorProcessor.Blob b : blobs) {
                    RotatedRect boxFit = b.getBoxFit();
                }
                MoveTo = blobs.get(0).getBoxFit().center;
                AngV = -Math.signum(MoveTo.x-pxX)/angS;
                AngV = min(AngV*Math.abs(MoveTo.x-pxX), 0.3);
                if (Math.abs(MoveTo.x-pxX) < 10) {
                    AngV = 0;
                }
                //AngV = AngPID.control(MoveTo.x);
                ForV = Math.signum(-(MoveTo.y-pxY))*min(max(0.2, Math.abs((MoveTo.y-pxY)/forS)), 0.4);
                if (Math.abs(MoveTo.y-pxY) < 10) {
                    ForV = 0;
                }
                telemetry.addData("MoveTo", MoveTo);
                telemetry.addData("Ang V", AngV);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                ForV,
                                0
                        ),
                        AngV
                ));
                if (ForV == 0 && AngV == 0) {
                    bend();
                    if (claw.getPosition() < 0.5) {return;}
                }
            } else {
                //AngPID = new PID(0.01, 0, 0.01, 160);
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                0
                        ),
                        0
                ));
            }
            telemetry.update();
            sleep(50);
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
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                //.setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                //.setTargetColorRange(ColorRange.RED)
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.9, 0, 0.9, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .lineToY(39)
                .turnTo(Math.toRadians(55))
                .lineToY(51)
                .strafeTo(new Vector2d(51, 51));
        Action Action1 = tab1.build();

        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(40, 30, Math.toRadians(270)))
                .lineToY(39)
                .turnTo(Math.toRadians(55))
                .lineToY(51)
                .strafeTo(new Vector2d(47, 58))
                .waitSeconds(0.5);
        Action ToBasket = tab8.build();
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
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(40, 39));
        Action Toblock1 = tab5.build();
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(45)))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(57, 37));
        Action Toblock2 = tab7.build();
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
        TrajectoryActionBuilder tab10 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(35, 40))
                .strafeTo(new Vector2d(35, 0))
                .turnTo(180)
                .strafeTo(new Vector2d(20, 0));
        Action ToBar = tab10.build();
        waitForStart();
        clawArm.setPosition(0);
        rotateSlide(45);
        if (isStopRequested()) return;
        Actions.runBlocking(Action1);

        if (isStopRequested()) return;
        sleep(100);
        reachBasket();
        if (isStopRequested()) return;
        //Actions.runBlocking(ToBar);
        //slide(-400);
        //clawArm.setPosition(0.4);

        Actions.runBlocking(Toblock1);
        grabBlock();
        if (isStopRequested()) return;
        Actions.runBlocking(ToBasket);
        if (isStopRequested()) return;
        reachBasket();
        if (isStopRequested()) return;
        Actions.runBlocking(ToBar);
    }
}