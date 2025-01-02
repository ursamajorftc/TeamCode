package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TwoGamepads", group = "Linear OpMode")
@Config
public class TwoGamepads extends LinearOpMode {

    // Hardware variables
    private DcMotor intakeDrive, outtakeDrive1, outtakeDrive2;
    private Servo intakeServoLeft, intakeServoRight, lockServo;
    private CRServo intakeCRSLeft, intakeCRSRight;
//    public class AnalogInput axonServo = hardwareMap.get(AnalogInput.class, "axonServo");
//    public class AnalogInput axonServo2 = hardwareMap.get(AnalogInput.class, "axonServo2");
    // Constants
    public static double INTAKE_SPIN_POWER = 1.0;
    public static double INTAKE_DOWN_LPOSITION = 0.58;
    public static double INTAKE_DOWN_RPOSITION = 0.4;
    public static double INTAKE_UP_LPOSITION = 0.18;
    public static double INTAKE_UP_RPOSITION = 0.8;
    public static double LOCK = -0.5;
    public static double UNLOCK = 0;
    public static int FULL_EXTENSION = 965; // this is for the horizontal slide(s) to extend fully
    public static int HALF_EXTENSION = 482; // 965 / 2
    private volatile boolean isRunning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Start threads
        Thread slideControlThread = new Thread(this::handleSlideControls);
        Thread intakeControlThread = new Thread(this::handleIntakeControls);
        Thread manualOverrideThread = new Thread(this::manualOverrideRetract);
        Thread halfExtendSlides = new Thread(this::halfExtendSlides);
        Thread fullExtendSlides = new Thread(this::FullExtendSlides);

        Thread retractSlides = new Thread(this::retractSlides);
        Thread moveIntakeDown = new Thread(this::moveIntakeDown);
        Thread moveIntakeUp = new Thread(this::moveIntakeUp);
        Thread activateIntake = new Thread(this::activateIntake);
        Thread backspinIntake = new Thread(this::backspinIntake);
        Thread stopAllMotors = new Thread(this::stopAllMotors);


        slideControlThread.start();
        intakeControlThread.start();
        manualOverrideThread.start();
        FullExtendSlides();
        halfExtendSlides.start();
        retractSlides.start();
        moveIntakeDown.start();
        moveIntakeUp.start();
        activateIntake.start();
        backspinIntake.start();
        stopAllMotors.start();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad1: Primary controls (e.g., drivetrain)
            // Gamepad2: Secondary controls (e.g., arm, intake, etc.)

            // debugging
            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            telemetry.addData("Outtake Motor 1 Pos", outtakeDrive1.getCurrentPosition());
            telemetry.addData("Outtake Motor 2 Pos", outtakeDrive2.getCurrentPosition());
            telemetry.update();
        }

        // Stop them threads when the opmode is stopped 
        isRunning = false;
        slideControlThread.join();
        intakeControlThread.join();
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
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleSlideControls() {
        while (isRunning) {
            if (gamepad1.right_trigger > 0.5) {
                FullExtendSlides();
                activateIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);
            }

            if (gamepad1.right_bumper) {
                halfExtendSlides();
                activateIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);
            }

            if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
                retractSlides();
                lockServo.setPosition(UNLOCK);
                activateIntake();
            }
        }
    }

    private void handleIntakeControls() {
        while (isRunning) {
            if (gamepad2.y) {
                backspinIntake();
            }

            if (gamepad2.b) {
                stopAllMotors();
            }

            if (gamepad2.x) {
                activateIntake();
            }

            if (gamepad2.dpad_down) {
                moveIntakeDown();
            }

            if (gamepad2.dpad_up) {
                moveIntakeUp();
            }
        }
    }

    private void halfExtendSlides() {
        intakeDrive.setTargetPosition(HALF_EXTENSION);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);
        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0);
    }
    private void FullExtendSlides() {
        intakeDrive.setTargetPosition(FULL_EXTENSION);
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
