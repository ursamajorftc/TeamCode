package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ImprovedTeleOp", group = "Linear OpMode")
@Config
@Disabled
public class maybe extends LinearOpMode {

    // Hardware variables
    private DcMotor intakeDrive, outtakeDrive1, outtakeDrive2;
    private Servo intakeServoLeft, intakeServoRight, armServo, fingerServo, wristServo, lockServo;
    private CRServo intakeCRSLeft, intakeCRSRight;

    // Constants
    public static double ARM_GRAB = -0.3;
    public static double WRIST_90 = 0.5;
    public static double WRIST_SCORE = -0.5;
    public static double FINGER_OPEN = 0.4;
    public static double FINGER_CLOSE = 0;
    public static int FULL_EXTENSION = 965;
    public static int HALF_EXTENSION = 482;
    public static double INTAKE_SPIN_POWER = 1.0;
    public static double INTAKE_DOWN_LPOSITION = 0.58;
    public static double INTAKE_DOWN_RPOSITION = 0.4;
    public static double INTAKE_UP_LPOSITION = 0.18;
    public static double INTAKE_UP_RPOSITION = 0.8;
    public static double OUTTAKE_UP_POSITION = 3800;
    public static double LOCK = -0.5;
    public static double UNLOCK = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain control
            controlDrivetrain();

            // Mechanism controls
            if (gamepad1.right_trigger > 0.5) {
                extendSlides(FULL_EXTENSION);
                activateIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);
            }

            if (gamepad1.right_bumper) {
                extendSlides(HALF_EXTENSION);
                activateIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);
            }

            if (gamepad1.left_trigger > 0.5) {
                retractSlides();
                lockServo.setPosition(UNLOCK);
                activateIntake();
            }

            if (gamepad1.y) {
                backspinIntake();
            }

            if (gamepad1.b) {
                scoreBlock(3500, 1.0);
            }
            if (gamepad1.dpad_down) {

                moveOuttakeSlides(20,-0.5);
            }
            if (gamepad1.dpad_up) {
                manualOverrideRetract();
            }

            // Telemetry for debugging
            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            telemetry.addData("Outtake Motor 1 Pos", outtakeDrive1.getCurrentPosition());
            telemetry.addData("Outtake Motor 2 Pos", outtakeDrive2.getCurrentPosition());
            telemetry.update();
        }

        stopAllMotors();
    }

    private void initializeHardware() {
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        outtakeDrive1 = hardwareMap.get(DcMotor.class, "outmoto1");
        outtakeDrive2 = hardwareMap.get(DcMotor.class, "outmoto2");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        armServo = hardwareMap.get(Servo.class, "armServo");
        fingerServo = hardwareMap.get(Servo.class, "fingerServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlDrivetrain() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x
        ));
    }

    private void extendSlides(int targetPosition) {
        intakeDrive.setTargetPosition(targetPosition);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);
        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0);
    }

    private void retractSlides() {
        moveIntakeUp();
        intakeDrive.setTargetPosition(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);
        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Retracting Slide", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0);
    }

    private void scoreBlock(int targetPosition, double maxPower) {

        moveOuttakeSlides(targetPosition, maxPower);


    }


    private void moveOuttakeSlides(int targetPosition, double maxPower) { // example: moveOuttakeSlides(3800,1)
        outtakeDrive1.setTargetPosition(targetPosition);
        outtakeDrive2.setTargetPosition(-targetPosition);

        outtakeDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeDrive1.setPower(maxPower);
        outtakeDrive2.setPower(maxPower);

        while (outtakeDrive1.isBusy() && outtakeDrive2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake 1 Position", outtakeDrive1.getCurrentPosition());
            telemetry.addData("Outtake 2 Position", outtakeDrive2.getCurrentPosition());
            telemetry.update();
        }
    }

    private void moveIntakeDown() {
        intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION);
        intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION);
    }

    private void moveIntakeUp() {
        intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
    }

    private void activateIntake() {
        intakeCRSLeft.setPower(-INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(INTAKE_SPIN_POWER);
    }

    private void backspinIntake() {
        intakeCRSLeft.setPower(INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(-INTAKE_SPIN_POWER);
    }

    private void stopAllMotors() {
        intakeDrive.setPower(0);
        outtakeDrive1.setPower(0);
        outtakeDrive2.setPower(0);
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
    private void manualOverrideRetract() {
        telemetry.addLine("Manual Override: Retracting Slides");
        telemetry.update();
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setPower(-1);
        intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);

        while (opModeIsActive() && gamepad1.left_bumper) {
            telemetry.addData("Manual Retraction", "Hold to Retract");
            telemetry.update();
        }
        intakeDrive.setPower(0);
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Manual Override Complete: Encoder Reset");
        telemetry.update();
    }
}
