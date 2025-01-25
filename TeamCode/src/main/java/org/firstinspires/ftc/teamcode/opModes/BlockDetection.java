package org.firstinspires.ftc.teamcode.opModes;


import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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


//@Disabled
@TeleOp(name = "TEST: Block Detect")
public class BlockDetection extends LinearOpMode
{
    private Point MoveTo;
    private MecanumDrive drive;
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));


    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, initialPose);
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
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
        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
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
                telemetry.addData("MoveTo", MoveTo);
                /*drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -(MoveTo.y-220)/80,
                                0
                        ),
                        (-Math.abs(MoveTo.x-160)/(MoveTo.x-160))/5
                ));*/
            } else {
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
}