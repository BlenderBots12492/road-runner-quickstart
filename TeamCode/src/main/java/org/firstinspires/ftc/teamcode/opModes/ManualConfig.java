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
    @Override
    public void runOpMode() {

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRotator = hardwareMap.get(DcMotor.class, "slideRotator");

        slideExtenderEnc = hardwareMap.get(DcMotor.class, "leftSlide");
        LimitSwitch = hardwareMap.get(DcMotor.class, "touchSwitch");
        slideRotatorEnc = hardwareMap.get(DcMotor.class, "slideRotator");

        slideRotator.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRotatorEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideExtenderEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            while (LimitSwitch.getCurrentPosition() == 0) {
                //slides extend / retract
                slideRotator.setPower(-0.5);
            }
            slideRotatorEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideExtenderEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRotatorEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideRotator.setTargetPosition((int) Math.round(45*0.0244));


        }
    }
}