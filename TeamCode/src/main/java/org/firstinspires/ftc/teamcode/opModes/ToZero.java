package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ToZero")
public class ToZero extends LinearOpMode {
    private DcMotor rightSlide;
    private DcMotor leftSlide;
    private DcMotor slideRotator;
    @Override
    public void runOpMode() {

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");

        slideRotator.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        slideRotator.setTargetPosition(0);
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRotator.setPower(1);
        while (slideRotator.isBusy()) { sleep(100); }
        slideRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}