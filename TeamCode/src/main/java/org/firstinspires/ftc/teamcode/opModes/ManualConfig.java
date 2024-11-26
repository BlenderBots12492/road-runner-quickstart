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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoConfig")
public class ManualConfig extends LinearOpMode {
    private DcMotor rightSlide;
    private DcMotor leftSlide;
    private DcMotor slideRotator;
    private DcMotor slideExtenderEnc;
    private DcMotor slideRotatorEnc;
    private DcMotor LimitSwitch;
    private Servo clawArm;
    @Override
    public void runOpMode() {

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");

        slideExtenderEnc = hardwareMap.get(DcMotor.class, "leftSlide");
        LimitSwitch = hardwareMap.get(DcMotor.class, "rightSlide");
        slideRotatorEnc = hardwareMap.get(DcMotor.class, "slideRotator");

        clawArm = hardwareMap.get(Servo.class, "clawArm");

        slideRotator.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotatorEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideExtenderEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        clawArm.setPosition(0);
        // Put run blocks here.
        slideRotator.setPower(-0.5);
        sleep(100);
        telemetry.addLine().addData("Switch", LimitSwitch.getCurrentPosition());
        telemetry.update();
        LimitSwitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (LimitSwitch.getCurrentPosition() == 0 && opModeIsActive()) {
            //slides extend / retract
            clawArm.setPosition(0);
            slideRotator.setPower(0.5);

            telemetry.addLine().addData("Switch", LimitSwitch.getCurrentPosition());
            telemetry.update();

        }
        slideRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtenderEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine().addData("test", slideRotator.getCurrentPosition());
        telemetry.update();

    }
}