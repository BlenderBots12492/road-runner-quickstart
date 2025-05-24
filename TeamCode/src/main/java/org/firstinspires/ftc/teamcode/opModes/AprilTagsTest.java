package org.firstinspires.ftc.teamcode.opModes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodAccess;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodCurrentGameAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import java.util.List;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTagsTest  extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(48, 48, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");


        //Position cameraPosition = new Position(DistanceUnit.INCH, -7, 8, 1, 0);
        Position cameraPosition = new Position(DistanceUnit.INCH, 8, -9.5, 9.5, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -90, -90, 90, 0);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
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

        while (opModeIsActive()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            List<AprilTagDetection> tags = tagProcessor.getDetections();
            if (tags.size() > 0) {
                AprilTagDetection tag = tags.get(0);
                telemetry.addData("Dist", tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.elevation);
                telemetry.addData("elevation", tag.ftcPose.bearing);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("PoseBot", tag.robotPose.getPosition());
                drive.updatePoseEstimate();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(tag.robotPose.getPosition().x, tag.robotPose.getPosition().y, Math.toRadians(tag.robotPose.getOrientation().getYaw())));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.addData("PoseBot-ENC", drive.pose);
                Position roboPos = tag.robotPose.getPosition();

                telemetry.update();

            } else {

            }
        }
    }
}
