package org.firstinspires.ftc.teamcode.opModes;
import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

import kotlin.reflect.KParameter;


//@Disabled
@Autonomous(name = "TEST: Block Detect")
public class BlockDetection extends LinearOpMode
{
    class PID {
        private double KP;
        private double KI;
        private double KD;
        private double point;
        private double lastError;
        private double integral;
        public PID(double Kp, double Ki, double Kd, double setPoint) {
            this.KP = Kp;
            this.KI = Ki;
            this.KD = Kd;
            this.point = setPoint;
        }
        private double control(double measurement) {
            double error = point - measurement;
            double derivative = error - lastError;
            integral += error;
            lastError = error;
            return KP * error + KI * integral + KD * derivative;
        }
    }
    private Point MoveTo;
    private MecanumDrive drive;
    private DcMotor slideRotator;

    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo clawArm;
    private Servo claw;
    public void slide(double distance) {
        while (opModeIsActive()) {
            if ((leftSlide.getCurrentPosition())+10 < distance) {
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
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));
    private void bend() {
        claw.setPosition(1);
        sleep(300);
        rotateSlide(0);
        slideRotator.setPower(0);
        claw.setPosition(0);
        sleep(300);
        slideRotator.setPower(1);
        sleep(1000);
        slideRotator.setPower(0);
    }
    private void reachBasket() {
        sleep(100);
        rotateSlide(65);
        if (isStopRequested()) return;
        slide(-2380);
        if (isStopRequested()) return;
        clawArm.setPosition(1);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        claw.setPosition(1);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        clawArm.setPosition(0);
        if (isStopRequested()) return;
        sleep(500);
        if (isStopRequested()) return;
        rotateSlide(67);
        if (isStopRequested()) return;
        slide(0);
        //if (isStopRequested()) return;
        //sleep(100);
        //if (isStopRequested()) return;
        //rotateSlide(45);
    }
    private void grabBlock() {
        double ForV;
        double AngV;
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
                AngV = -Math.signum(MoveTo.x-160)/200;
                AngV = min(AngV*Math.abs(MoveTo.x-160), 0.3);
                if (Math.abs(MoveTo.x-160) < 10) {
                    AngV = 0;
                }
                //AngV = AngPID.control(MoveTo.x);

                ForV = Math.signum(-(MoveTo.y-230))*min(max(0.2, Math.abs((MoveTo.y-230)/100)), 0.4);
                if (Math.abs(MoveTo.y-230) < 10) {
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
                    return;
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
    private ColorBlobLocatorProcessor colorLocator;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        claw = hardwareMap.get(Servo.class, "claw");
        clawArm = hardwareMap.get(Servo.class, "clawArm");

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
                //.splineTo(new Vector2d(40, 50), Math.toRadians(270))
                //.splineTo(new Vector2d(47, 47), Math.toRadians(45))
                .waitSeconds(1)
                .lineToY(39)
                .turnTo(Math.toRadians(55))
                .strafeTo(new Vector2d(50, 50));
        Action ToBasket1 = tab1.build();

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(49, 49, Math.toRadians(50)))
                .strafeTo(new Vector2d(48, 35))
                .turnTo(200);
        Action ToBlock1 = tab2.build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        waitForStart();
        //PID AngPID = new PID(0.01, 0, 0.01, 160);
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        grabBlock();
        /*Actions.runBlocking(ToBasket1);
        reachBasket();
        Actions.runBlocking(ToBlock1);
        grabBlock();
        Actions.runBlocking(ToBasket1);
        reachBasket();*/
    }
}