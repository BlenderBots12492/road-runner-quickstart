package org.firstinspires.ftc.teamcode.opModes;


import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
@TeleOp(name = "TEST: Block Detect")
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
    private Servo claw;
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));
    private void bend() {
        double distance = 0;
        claw.setPosition(1);
        sleep(300);
        while (opModeIsActive()) {
            if (slideRotator.getCurrentPosition() + 10 < distance) {
                slideRotator.setPower(1);
            } else if (slideRotator.getCurrentPosition() - 10 > distance) {
                slideRotator.setPower(-1);
            } else {
                slideRotator.setPower(0);
                claw.setPosition(0);
                sleep(300);
                slideRotator.setPower(1);
                sleep(1000);
                slideRotator.setPower(0);
                return;
            }
        }
    }
    private void grabBlock() {
        double ForV;
        double AngV;
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();


            ColorBlobLocatorProcessor.Util.filterByArea(100, 20000, blobs);  // filter out very small blobs.

            telemetry.addLine(" Area Density Aspect  Center");

            // Display the size (area) and center location for each Blob.
            if (!blobs.isEmpty()) {
                for (ColorBlobLocatorProcessor.Blob b : blobs) {
                    RotatedRect boxFit = b.getBoxFit();
                }
                MoveTo = blobs.get(0).getBoxFit().center;
                AngV = -Math.signum(MoveTo.x-160)/200;
                AngV = Math.min(AngV*Math.abs(MoveTo.x-160), 0.3);
                if (Math.abs(MoveTo.x-160) < 10) {
                    AngV = 0;
                }
                //AngV = AngPID.control(MoveTo.x);

                ForV = -(MoveTo.y-220)/120;
                if (Math.abs(MoveTo.y-220) < 10) {
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
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");
        claw = hardwareMap.get(Servo.class, "claw");

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                //.setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                //.setTargetColorRange(ColorRange.RED)
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.9, 0.5, 0.9, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        waitForStart();
        double AngV;
        double ForV = 0;
        //PID AngPID = new PID(0.01, 0, 0.01, 160);
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        grabBlock();
    }
}