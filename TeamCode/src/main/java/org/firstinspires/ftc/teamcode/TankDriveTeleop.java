package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TankDriveTeleOp", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
public class TankDriveTeleop extends LinearOpMode {


    private DcMotor rightfrontMotor;
    private DcMotor leftfrontMotor;
    private DcMotor pinionliftMotor;

    private DcMotor[] motors;

    @Override
    public void runOpMode() {

        rightfrontMotor = hardwareMap.dcMotor.get("rightfront");
        leftfrontMotor = hardwareMap.dcMotor.get("leftfront");
        pinionliftMotor = hardwareMap.dcMotor.get("pinionliftMotor");
//        rightbackMotor = hardwareMap.dcMotor.get("rightback");
//        leftbackMotor = hardwareMap.dcMotor.get("leftback");

//        leftbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightbackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightfrontMotor.setDirection(DcMotor.Direction.FORWARD);
//        leftbackMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightbackMotor.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {

            double slowFactor;

            if (gamepad1.left_bumper) {
                slowFactor = 0.35;
            } else {
                slowFactor = 1;
            }

            leftfrontMotor.setPower(gamepad1.left_stick_y * slowFactor);
            rightfrontMotor.setPower(gamepad1.right_stick_y * slowFactor);
//            leftbackMotor.setPower(gamepad1.left_stick_y * slowFactor);
//            rightbackMotor.setPower(gamepad1.right_stick_y * slowFactor);
        }
    }
}
