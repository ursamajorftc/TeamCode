package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpButBetter", group = "Linear OpMode")
@Config
@Disabled
public class TeleOpUntst extends LinearOpMode {


    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;


    final int FULL_EXTENSION = 965;
    final int HALF_EXTENSION = FULL_EXTENSION / 2;
    final int QUARTER_EXTENSION = FULL_EXTENSION / 3;



    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware and PIDF initialization
        initHardware();


        // Initialize telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        int targetPosition = 0; // Track the desired position

        while (opModeIsActive()) {
            // Drive controls
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));

            // IntakeDrive controls

            if (gamepad1.right_trigger > 0.5) {
                moveToPosition(FULL_EXTENSION);
            }
            // Half extension
            else if (gamepad1.right_bumper) {
                moveToPosition(HALF_EXTENSION);
            }
            // Quarter extension
            else if (gamepad1.y) {
                moveToPosition(QUARTER_EXTENSION);
            }
            // Retract completely
            else if (gamepad1.x) {
                moveToPosition(0);
            }

            if (gamepad1.dpad_up) {
                intakeServoLeft.setPosition(1.0);
                intakeServoRight.setPosition(0.0);
            }

            // Reset intake position
            if (gamepad1.dpad_down) {
                intakeServoLeft.setPosition(0.0);
                intakeServoRight.setPosition(1.0);
            }
            // Intake Control
            if (gamepad1.a) {
                spinIntake(-1.0); // Move intake down
            } else if (gamepad1.b) {
                spinIntake(1.0); // Move intake up
            } else {
                spinIntake(0.0); // Stop intake
            }

            // Update telemetry
            updateTelemetry(drive, dashboard, targetPosition);
        }
    }

    private void moveToPosition(int targetPosition) {
        intakeDrive.setTargetPosition(targetPosition);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(0.8); // Adjust power to prevent overheating
        while (opModeIsActive() && intakeDrive.isBusy()) {
            // Wait until the motor reaches the target position
        }
        intakeDrive.setPower(0.0); // Stop the motor
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void spinIntake(double power) {
        intakeCRSLeft.setPower(power);
        intakeCRSRight.setPower(-power);
    }
    private void positionIntake (double intakePosition) {
        intakeServoLeft.setPosition(intakePosition);
        intakeServoRight.setPosition(-intakePosition);


    }

    // Telemetry updates
    private void updateTelemetry(MecanumDrive drive, FtcDashboard dashboard, int targetPosition) {
        drive.updatePoseEstimate();
        double intakePosition = intakeDrive.getCurrentPosition();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("Intake Position", intakePosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Motor Power", intakeDrive.getPower());
        telemetry.addData("Servo Left Position", intakeServoLeft.getPosition());
        telemetry.addData("Servo Right Position", intakeServoRight.getPosition());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", drive.pose.position.x);
        packet.put("y", drive.pose.position.y);
        packet.put("heading", Math.toDegrees(drive.pose.heading.toDouble()));
        packet.put("Intake Position", intakePosition);
        packet.put("Target Position", targetPosition);
        packet.put("Motor Power", intakeDrive.getPower());
        dashboard.sendTelemetryPacket(packet);
    }

    // Hardware initialization
    private void initHardware() {
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
    }}
