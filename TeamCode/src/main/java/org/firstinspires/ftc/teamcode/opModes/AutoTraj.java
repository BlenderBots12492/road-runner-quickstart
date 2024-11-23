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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoTraj", group = "Concept")
public class AutoTraj extends LinearOpMode {
    private Pose2d initialPose = new Pose2d(38, 61.7, Math.toRadians(270));

    private MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
    private DcMotor leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
    private DcMotor rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
    private Servo claw = hardwareMap.get(Servo.class, "claw");

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            while (true) {
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                if (leftSlide.getCurrentPosition() >= 10000) {
                    leftSlide.setPower(0);
                    rightSlide.setPower(0);
                    return(true);
                }
            }
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }
    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(0);
            return(true);
        }
    }
    public Action ClawClose() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(1);
            return(true);
        }
    }
    public Action ClawOpen() {
        return new ClawOpen();
    }
    public void runOpMode() {
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(38, 38), Math.toRadians(270))
                .splineTo(new Vector2d(60, 61.7), Math.toRadians(45))
                .waitSeconds(1);

        Action Action1 = tab1.build();

        if (isStopRequested()) return;

        Actions.runBlocking(Action1);
    }
}
