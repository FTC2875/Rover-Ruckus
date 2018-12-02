package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "BuddyBot Teleop", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
public class BuddyBotTeleop extends LinearOpMode {

    private DcMotor leftDriveMotor;
    private DcMotor rightDriveMotor;
    private DcMotor spinner;

    private CRServo lift;
    private Servo bucketFlipper;

    private boolean buttonPressed = false;

    @Override
    public void runOpMode() {

        rightDriveMotor = hardwareMap.dcMotor.get("rightdrive");
        leftDriveMotor = hardwareMap.dcMotor.get("leftdrive");
        spinner = hardwareMap.dcMotor.get("spinner");
        lift = hardwareMap.crservo.get("lift");
        bucketFlipper = hardwareMap.servo.get("flipper");

        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double slowFactor;

            if (gamepad1.left_bumper) {
                slowFactor = 0.35;
            } else {
                slowFactor = 1;
            }

            // lift mechanism
            if (gamepad1.dpad_up) {
                lift.setPower(0.75);

                telemetry.addData("servo pos: ", lift.getController().getServoPosition(1));
                telemetry.update();
            } else if (gamepad1.dpad_down) {
                lift.setPower(-0.75);

                telemetry.addData("servo pos: ", lift.getController().getServoPosition(1));
                telemetry.update();
            } else {
                lift.setPower(0);
            }

            // spinner mechanism
            if (gamepad1.dpad_left) {
                spinner.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                spinner.setPower(-0.5);
            } else {
                spinner.setPower(0);
            }

            //flipper mechanism
            if (gamepad1.y) {
                double curPos = bucketFlipper.getPosition();
                double newPos = curPos + 0.005;

                telemetry.addData("new pos: ", newPos);
                telemetry.addData("cur pos: ", curPos);
                telemetry.update();

                if (newPos < 1)
                    bucketFlipper.setPosition(newPos);

            } else if (gamepad1.a) {
                double curPos = bucketFlipper.getPosition();
                double newPos = curPos - 0.005;

                telemetry.addData("new pos: ", newPos);
                telemetry.addData("cur pos: ", curPos);
                telemetry.update();

                if (newPos > 0.1)
                    bucketFlipper.setPosition(newPos);

            }

            buttonPressed = gamepad1.y || gamepad1.a;

            leftDriveMotor.setPower(-gamepad1.left_stick_y * slowFactor);
            rightDriveMotor.setPower(-gamepad1.right_stick_y * slowFactor);

            telemetry.addData("servo pos", bucketFlipper.getPosition());
            telemetry.update();


        }
    }
}
