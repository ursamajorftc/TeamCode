package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.threadopmode.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "guh_edited", group = "Linear OpMode")
@Config
public class TestingThreads extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft, intakeServoRight, armServo, clawServo, wristServo, lockServo;
    private CRServo intakeCRSLeft, intakeCRSRight;

    // Public static variables for editable positions
    public static double ARM_GRAB = -0.3; // Arm grab position
    public static double WRIST_GRAB = 0.5; // Wrist grab position
    public static double WRIST_SCORE = -0.5; // Wrist score position
    public static double FINGER_OPEN = 0.4; // Finger open position
    public static double FINGER_CLOSE = 0.0; // Finger close position

    public static double HORIZONTAL_SLIDE_INTAKE_DOWN_LEFT = 0.58;
    public static double HORIZONTAL_SLIDE_INTAKE_DOWN_RIGHT = 0.4;
    public static double HORIZONTAL_SLIDE_INTAKE_UP_LEFT = 0.18;
    public static double HORIZONTAL_SLIDE_INTAKE_UP_RIGHT = 0.8;
    public static double HORIZONTAL_SLIDE_OUT = 965;
    public static double HORIZONTAL_SLIDE_IN = 0;
    public static double ARMGRAB = -0.3; // arm at 90 degrees parallel with the intake
    public static double WRIST90 = 0.5; // pos for grabby grabby block
    public static double FINGEROPEN = 0.4; // pos for open finger
    public static double FINGERCLOSE = 0; // pos for close finger
    public static int ARMVERTICAL = 0; // VERTICAL WITH RAILS
    //raise the rails
    public static double WRISTSCORE = -0.5; // funny angle for the drop
    // and then finger opens
    public static double INTAKE_SPIN_POWER = 1.0; // Power for intake spinning
    public static double LOCK = -0.5; // Servo lock position
    public static double UNLOCK = 0.0; // Servo unlock position

    private volatile boolean isRunning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        initializeHardware();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        // Threads
        Thread intakeControlThread = new Thread(this::handleIntakeControls);
        Thread servoControlThread = new Thread(this::handleServoControls);

        intakeControlThread.start();
        servoControlThread.start();

        // Driving logic in main loop
        while (opModeIsActive()) {
            handleDriveControls();
            telemetry.update();
        }

        // Stop threads on OpMode end
        isRunning = false;
        intakeControlThread.join();
        servoControlThread.join();
    }

    private void initializeHardware() {
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleDriveControls() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x
        ));
    }

    private void handleIntakeControls() {
        while (isRunning && opModeIsActive()) {
            if (gamepad1.y) {
                backspin();
            } else if (gamepad1.b) {
                spinIntake();
            } else {
                stopIntake();
            }
        }
    }

    private void handleServoControls() {
        while (isRunning && opModeIsActive()) {
            if (gamepad1.a) {
                moveIntakeDown();
            } else if (gamepad1.x) {
                retractSlide();
            } else if (gamepad1.left_bumper) {
                lockServo.setPosition(LOCK);
            } else if (gamepad1.right_bumper) {
                lockServo.setPosition(UNLOCK);
            }
        }
    }

    private void retractSlide() {
        intakeServoLeft.setPosition(HORIZONTAL_SLIDE_INTAKE_UP_LEFT);
        intakeServoRight.setPosition(HORIZONTAL_SLIDE_INTAKE_UP_RIGHT);
        intakeCRSLeft.setPower(-0.025);
        intakeCRSRight.setPower(0.025);
        intakeDrive.setPower(0.0);
    }

    private void moveIntakeDown() {
        intakeServoLeft.setPosition(HORIZONTAL_SLIDE_INTAKE_DOWN_LEFT);
        intakeServoRight.setPosition(HORIZONTAL_SLIDE_INTAKE_DOWN_RIGHT);
    }

    private void spinIntake() {
        intakeCRSLeft.setPower(-INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(INTAKE_SPIN_POWER);
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
        intakeDrive.setTargetPosition(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);
        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Retracting Slide", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0);
    }

    private void backspin() {
        intakeCRSLeft.setPower(INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(-INTAKE_SPIN_POWER);
    }

    private void stopIntake() {
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
}
