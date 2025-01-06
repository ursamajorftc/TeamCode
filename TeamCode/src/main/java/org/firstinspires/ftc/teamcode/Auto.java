// Import necessary libraries for FTC
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Auto extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft, intakeServoRight;
    private CRServo intakeCRSLeft, intakeCRSRight;
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

    private ElapsedTime runtime = new ElapsedTime();

    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        // Set motor directions (if necessary)
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeCRSLeft.setDirection(CRServo.Direction.FORWARD);
        intakeCRSRight.setDirection(CRServo.Direction.REVERSE);

        // Set the dashboard telemetry
        dashboardTelemetry = telemetry;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start autonomous mode if selected
        if (opModeIsActive()) {
            autonomousMode();
        }

        // Start teleop mode if selected
        if (opModeIsActive()) {
            teleopMode();
        }
    }

    // Autonomous Mode logic
    private void autonomousMode() {
        // Use runtime for time-based operations during autonomous
        runtime.reset();

        // Example of autonomous operations (move intake motors, open/close claw)
        intakeDrive.setPower(INTAKE_SPIN_POWER);  // Move intake at full power
        intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION);  // Set intakeServoLeft to a specific position
        intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION); // Set intakeServoRight to a specific position
        clawServo.setPosition(CLAW_OPEN);  // Open claw servo (0.4 can be open, adjust per design)

        // Send data to the FTC Dashboard
        dashboardTelemetry.addData("Autonomous Mode", "Running");
        dashboardTelemetry.addData("Intake Motor Power", intakeDrive.getPower());
        dashboardTelemetry.addData("Left Intake Servo Position", intakeServoLeft.getPosition());
        dashboardTelemetry.addData("Right Intake Servo Position", intakeServoRight.getPosition());
        dashboardTelemetry.addData("Claw Servo Position", clawServo.getPosition());
        dashboardTelemetry.update();

        // Autonomous example: Wait for 3 seconds then stop
        while (opModeIsActive()) {
        }

        // Stop intake motors after the autonomous actions are done
        intakeDrive.setPower(0);
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);

        // Update the dashboard once more after actions are completed
        dashboardTelemetry.addData("Autonomous Mode", "Completed");
        dashboardTelemetry.addData("Intake Motor Power", intakeDrive.getPower());
        dashboardTelemetry.update();
    }

    // Teleop Mode logic
    private void teleopMode() {
        // Use gamepad inputs to control robot hardware (example for intake, claw)
        while (opModeIsActive()) {
            // Intake control using gamepad buttons (example)
            if (gamepad1.a) {
                intakeDrive.setPower(INTAKE_SPIN_POWER); // Activate intake motor
            } else if (gamepad1.b) {
                intakeDrive.setPower(-INTAKE_SPIN_POWER); // Reverse intake motor
            } else {
                intakeDrive.setPower(0); // Stop intake motor
            }

            // Control intake servo (e.g., open/close the intake mechanism)
            if (gamepad1.x) {
                intakeServoLeft.setPosition(INTAKE_UP_LPOSITION); // Set intakeServoLeft to up position
                intakeServoRight.setPosition(INTAKE_UP_RPOSITION); // Set intakeServoRight to up position
            } else if (gamepad1.y) {
                intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION); // Set intakeServoLeft to down position
                intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION); // Set intakeServoRight to down position
            }

            // Control claw servo (open/close the claw)
            if (gamepad2.a) {
                clawServo.setPosition(CLAW_OPEN); // Open the claw
            } else if (gamepad2.b) {
                clawServo.setPosition(CLAW_CLOSE); // Close the claw
            }

            // Optional: Control CRServos for continuous intake or outtake
            if (gamepad1.left_bumper) {
                intakeCRSLeft.setPower(1.0); // Continuous intake on the left
                intakeCRSRight.setPower(1.0); // Continuous intake on the right
            } else if (gamepad1.right_bumper) {
                intakeCRSLeft.setPower(-1.0); // Reverse intake on the left
                intakeCRSRight.setPower(-1.0); // Reverse intake on the right
            } else {
                intakeCRSLeft.setPower(0); // Stop intake
                intakeCRSRight.setPower(0); // Stop intake
            }

            // Update the dashboard telemetry with real-time information
            dashboardTelemetry.addData("Teleop Mode", "Running");
            dashboardTelemetry.addData("Intake Motor Power", intakeDrive.getPower());
            dashboardTelemetry.addData("Left Intake Servo Position", intakeServoLeft.getPosition());
            dashboardTelemetry.addData("Right Intake Servo Position", intakeServoRight.getPosition());
            dashboardTelemetry.addData("Claw Servo Position", clawServo.getPosition());
            dashboardTelemetry.update();
        }
    }
}
