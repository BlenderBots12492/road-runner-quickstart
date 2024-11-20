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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "AutoTraj", group = "Concept")
public class AutoTraj extends LinearOpMode {

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        int heading = 0;
        int x1 = 0;
        int y1 = 10;
        int x2 = 10;
        int y2 = 10;
        Action tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(x1, y1), heading)
                .waitSeconds(1)
                .splineTo(new Vector2d(x2, y2), heading)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.

        while (!isStopRequested() && !opModeIsActive()) {
            int position = 1;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = 1;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1;

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}
