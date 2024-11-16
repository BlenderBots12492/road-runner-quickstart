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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="IntoTheDeepTeleOp")
public class TeleOp extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor rightSlide;
    private DcMotor leftSlide;
    private DcMotor slideRotator;
    private Servo clawWrist;
    private Servo claw;
    private Servo clawArm;
    private ElapsedTime runtime = new ElapsedTime();
    private double gamepad1_leftstick_x;
    private double gamepad1_leftstick_y;
    private double gamepad1_rightstick_x;
    private double gamepad1_rightstick_y;
    private DcMotor slideExtenderEnc;
    private DcMotor slideRotatorEnc;
    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");

        slideExtenderEnc = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRotatorEnc = hardwareMap.get(DcMotor.class, "slideRotator");

        claw = hardwareMap.get(Servo.class, "claw");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawArm = hardwareMap.get(Servo.class, "clawArm");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotator.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
        slideRotatorEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtenderEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         */
        slideRotatorEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideExtenderEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        Encoder slideRotatorEnc1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "slideRotator")));
        Encoder slideExtention = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftSlide")));

        double clawPos = 0.4;
        double clawWristPos = 0;
        double clawArmPos = 0.47;


        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                //slides extend / retract
                PositionVelocityPair slideExtVal = slideExtention.getPositionAndVelocity();
                PositionVelocityPair slideRotVal = slideRotatorEnc1.getPositionAndVelocity();
                /*if (slideExtVal.position*Math.cos(slideRotVal.position*0.5) > 1000) {
                    leftSlide.setPower(-1);
                    rightSlide.setPower(-1);
                } else*/ if (gamepad2.left_stick_y == 0) {
                    leftSlide.setPower(0.06);
                    rightSlide.setPower(0.06);
                } else if (/*slideExtVal.position*Math.cos(slideRotVal.position*0.5) < 1000*/true) {
                    leftSlide.setPower(-gamepad2.left_stick_y);
                    rightSlide.setPower(-gamepad2.left_stick_y);
                }
                /*
                if (gamepad2.left_stick_y == 0) {
                    leftSlide.setPower(0.06);
                    rightSlide.setPower(0.06);
                } else if (Math.cos(slideRotator.getCurrentPosition()/(18.9))*leftSlide.getCurrentPosition() < 100) { //1700:90
                    leftSlide.setPower(-gamepad2.left_stick_y);
                    rightSlide.setPower(-gamepad2.left_stick_y);
                } else if (-gamepad2.left_stick_y < 0) {
                    leftSlide.setPower(-gamepad2.left_stick_y);
                    rightSlide.setPower(-gamepad2.left_stick_y);
                }*/
                //slides rotate
                if (gamepad2.right_stick_y == 0) {
                    slideRotator.setPower(0);
                } else {
                    slideRotator.setPower(-gamepad2.right_stick_y);
                }


                //Get Claw Data
                //if (gamepad2.right_stick_x > 0.1 && clawArmPos < 0.48 ) { clawArmPos += 0.00005; }
                //if (gamepad2.right_stick_x < -0.1 && clawArmPos > 0.47 ) { clawArmPos -= 0.00005; }
                if (gamepad2.right_stick_x < -0.1 && clawWristPos > 0 ) { clawWristPos -= 0.004; }
                if (gamepad2.right_stick_x > 0.1 && clawWristPos < 1 ) { clawWristPos += 0.004; }

                if (gamepad2.left_stick_x != 0) {
                    clawArm.setPosition((gamepad2.left_stick_x)/2);
                }

                clawWrist.setPosition(clawWristPos);

                if (gamepad2.right_trigger > 0.1 && clawPos < 0.6 ) { clawPos += 0.05; }
                if (gamepad2.right_bumper && clawPos > 0.3 ) { clawPos -= 0.05; }
                claw.setPosition(clawPos);

                // here we are defining the variables for the gamepad motor powers
                if (0.5 > gamepad1.right_trigger) {
                    gamepad1_leftstick_x = -1 * gamepad1.left_stick_x;
                    gamepad1_leftstick_y = -1 * gamepad1.left_stick_y;
                    gamepad1_rightstick_x = gamepad1.right_stick_x;
                    gamepad1_rightstick_y = gamepad1.right_stick_y;
                } else {
                    gamepad1_leftstick_x = -0.4 * gamepad1.left_stick_x;
                    gamepad1_leftstick_y = -0.4 * gamepad1.left_stick_y;
                    gamepad1_rightstick_x = 0.4 * gamepad1.right_stick_x;
                    gamepad1_rightstick_y = 0.4 * gamepad1.right_stick_y;
                }
                // here we are defining the variables for the drive system
                rightFront.setPower(gamepad1_leftstick_y + gamepad1_leftstick_x - gamepad1_rightstick_x);
                rightBack.setPower(gamepad1_leftstick_y - gamepad1_leftstick_x - gamepad1_rightstick_x);
                leftFront.setPower(gamepad1_leftstick_y - gamepad1_leftstick_x + gamepad1_rightstick_x);
                leftBack.setPower(gamepad1_leftstick_y + gamepad1_leftstick_x + gamepad1_rightstick_x);

                telemetry.addLine().addData("SlidePos", slideExtVal.position);
                telemetry.addLine().addData("SlideAng", slideRotVal.position);
                telemetry.addLine().addData("SlidePos2", slideExtenderEnc.getCurrentPosition());
                telemetry.addLine().addData("SlideAng2", slideRotatorEnc.getCurrentPosition());
                telemetry.addLine();
                telemetry.addLine().addData("slideExtenderEnc.isBusy()", slideExtenderEnc.isBusy());

                telemetry.addLine().addData("test", leftFront.getCurrentPosition());

                telemetry.update();

            }
        }
    }
}