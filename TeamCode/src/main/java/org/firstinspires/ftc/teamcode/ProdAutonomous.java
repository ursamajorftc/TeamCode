package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "EXPDASHAutonomous", group = "Linear OpMode")
@Config
public class ProdAutonomous extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    private Servo clawServo;

    public static int FULL_EXTENSION = 965;
    public static int HALF_EXTENSION = 482;
    public static int QUARTER_EXTENSION = 241;
    public static double INTAKE_DOWN_LPOSITION = 0.58;
    public static double INTAKE_DOWN_RPOSITION = 0.4;
    public static double INTAKE_UP_LPOSITION = 0.18;
    public static double INTAKE_UP_RPOSITION = 0.8;
    public static double INTAKE_SPIN_POWER = 1.0;
    public static double CLAW_CLOSE = 0.0;
    public static double CLAW_OPEN = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Configuring motors
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Autonomous sequence starts here
        if (opModeIsActive()) {
            telemetry.addData("Autonomous Mode", "Starting sequence...");
            telemetry.update();

            // Step 1: Move the slide to FULL_EXTENSION and spin intake
            moveSlideToPosition(FULL_EXTENSION);
            spinIntake();
            moveIntakeDown();

            // Step 2: Open claw to release object
            clawServo.setPosition(CLAW_OPEN);
            sleep(1000); // Allow claw to open fully

            // Step 3: Retract slide
            retractSlide();

            // Step 4: Close claw to prepare for next object
            clawServo.setPosition(CLAW_CLOSE);
            sleep(1000);

            // Step 5: Move to HALF_EXTENSION for a mid-level scoring action
            moveSlideToPosition(HALF_EXTENSION);
            moveIntakeDown();
            spinIntake();

            // Step 6: Backspin intake to eject another object
            Backspin();
            sleep(1000); // Run backspin for 1 second
            stopIntake();

            // Step 7: Retract slide fully
            retractSlide();

            telemetry.addData("Autonomous Mode", "Sequence Complete");
            telemetry.update();
        }
    }

    private void moveSlideToPosition(int position) {
        intakeDrive.setTargetPosition(position);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);
        while (intakeDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving to position:", position);
            telemetry.addData("Current position:", intakeDrive.getCurrentPosition());
            telemetry.update();
        }
        intakeDrive.setPower(0.0);
    }

    private void retractSlide() {
        intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
        intakeCRSLeft.setPower(-0.025);
        intakeCRSRight.setPower(0.025);
        intakeDrive.setPower(1);
        moveSlideToPosition(0);
        intakeDrive.setPower(0.0);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void moveIntakeDown() {
        intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION);
        intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION);
    }

    private void spinIntake() {
        intakeCRSLeft.setPower(-INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(INTAKE_SPIN_POWER); // reverse to avoid servo damage
    }

    private void Backspin() {
        intakeCRSLeft.setPower(INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(-INTAKE_SPIN_POWER);
    }

    private void stopIntake() {
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
}
