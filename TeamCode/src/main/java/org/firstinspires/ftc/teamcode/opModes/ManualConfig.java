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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ManualConfig")
public class ManualConfig extends LinearOpMode {
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

        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                //slides extend / retract
                if (gamepad2.left_stick_y == 0) {
                    leftSlide.setPower(0.06);
                    rightSlide.setPower(0.06);
                } else {
                    leftSlide.setPower(-gamepad2.left_stick_y);
                    rightSlide.setPower(-gamepad2.left_stick_y);
                }
                //slides rotate
                if (gamepad2.right_stick_y == 0) {
                    slideRotator.setPower(0);
                } else {
                    slideRotator.setPower(-gamepad2.right_stick_y);
                }
                if (gamepad2.a) {
                    slideRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slideRotator.setTargetPosition((int) Math.round(-45/0.0244));
                    slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRotator.setPower(1);
                    while (slideRotator.isBusy()) { sleep(100); }
                    slideRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
    }
}