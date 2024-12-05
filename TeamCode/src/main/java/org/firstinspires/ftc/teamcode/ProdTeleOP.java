package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "EXPDASHTeleOp", group = "Linear OpMode")
@Config
public class ProdTeleOP extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    // CONFIG REQUIRED BEFORE YOU RUN THIS OR DAMAGE MAY OCCUR. These are variables "exposed" to FTCDashboard. USE FTC DASH TO CHANGE THESE, UNLESS YOU HAVE FIGURED OUT THESE VALUES - aaron 12:48 AM 12/5/24
    private static final int FULL_EXTENSION = 965;
    private static final int HALF_EXTENSION = 482;
    private static final int QUARTER_EXTENSION = 241;
    private static final double INTAKE_DOWN_LPOSITION = 0.0;
    private static final double INTAKE_DOWN_RPOSITION = 0.0;
    private static final double INTAKE_UP_LPOSITION = 0.0;
    private static final double INTAKE_UP_RPOSITION = 0.0;
    private static final double INTAKE_SPIN_POWER = 1.0;
    // end of exposed variables
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Config motor
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.5) {
                moveSlideToPosition(FULL_EXTENSION);
            } else if (gamepad1.right_bumper) {
                moveSlideToPosition(HALF_EXTENSION);
            } else if (gamepad1.y) {
                moveSlideToPosition(QUARTER_EXTENSION);
            } else if (gamepad1.x) {
                retractSlide();
            }


            // Automatically move the intake down and spin
            moveIntakeDown();
            spinIntake();

            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
    }

    private void moveSlideToPosition(int position) {
        intakeDrive.setTargetPosition(position);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);

        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("I want to move to:", position);
            telemetry.addData("I am at: ", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0.0);
    }

    private void retractSlide() {
        stopIntake();
        moveIntakeUp();
        intakeDrive.setTargetPosition(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(0.0);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void moveIntakeDown() {
        intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION);
        intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION);
    }
    private void moveIntakeUp() {
        intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
    }

    private void spinIntake() {
        intakeCRSLeft.setPower(INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(-INTAKE_SPIN_POWER); // reverse this so we don't explode servos lol
    }
    private void stopIntake() {
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
}
