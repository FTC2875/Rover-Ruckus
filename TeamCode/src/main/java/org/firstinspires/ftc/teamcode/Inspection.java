package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Inspection", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
public class Inspection extends LinearOpMode {

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;
    private DcMotor spinner;
    private DcMotor clamp;

    private DcMotor lift;
    private CRServo bucketFlipper;

    private boolean buttonPressed = false;

    private static final int MAX_LIFT = 3850;
    private static final int MIN_LIFT = 200;

    @Override
    public void runOpMode() {

        clamp = hardwareMap.dcMotor.get("clamp");


        waitForStart();

        double servPow = 0.41;

        while (opModeIsActive()) {


            // clamp mechanism
            if (gamepad2.y) {
                clamp.setPower(1);
            } else if(gamepad2.a) {
                clamp.setPower(-1);
            } else {
                clamp.setPower(0);
            }


            telemetry.update();


        }
    }
}
