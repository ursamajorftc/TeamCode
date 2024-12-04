package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Linear OpMode")
@Config
public class TeleOpUntst extends LinearOpMode {


    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;


    public static double P = 10.0;
    public static double I = 3.0;
    public static double D = 0.0;
    public static double F = 1.0;

    private PIDFController intakePIDF;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();


        intakePIDF = new PIDFController(P, I, D, F);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();


        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            runWithMecanumDrive(dashboard);
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            runWithTankDrive(dashboard);
        } else {
            throw new RuntimeException("Unknown drive class!");
        }
    }

    private void initHardware() {
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
    }

    private void runWithMecanumDrive(FtcDashboard dashboard) throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            intakePIDF.setPIDF(P, I, D, F);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (gamepad1.x) {
                controlIntakeDrive(965); // Move intake to position 965
            } else if (gamepad1.y) {
                controlIntakeDrive(0); // Reset intake to position 0
            }

            // Servo control
            if (gamepad1.a) {
                intakeServoLeft.setPosition(0);
            }

            updateTelemetry(drive, dashboard);
        }
    }

    private void runWithTankDrive(FtcDashboard dashboard) throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            // Drive logic
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            0.0
                    ),
                    -gamepad1.right_stick_x
            ));


            if (gamepad1.x) {
                controlIntakeDrive(965);
            } else if (gamepad1.y) {
                controlIntakeDrive(0);
            }

            // Update telemetry and dashboard
            updateTelemetry(drive, dashboard);
        }
    }

    private void controlIntakeDrive(int targetPosition) {
        int currentPosition = intakeDrive.getCurrentPosition();
        intakePIDF.setSetPoint(targetPosition);
        double pidfOutput = intakePIDF.calculate(currentPosition);
        intakeDrive.setPower(pidfOutput);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Intake Current Position", currentPosition);
        packet.put("Intake Target Position", targetPosition);
        packet.put("PIDF Output", pidfOutput);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private void updateTelemetry(MecanumDrive drive, FtcDashboard dashboard) {
        drive.updatePoseEstimate();
        double leftServoPosition = intakeServoLeft.getPosition();
        double rightServoPosition = intakeServoRight.getPosition();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("intakePosition", intakeDrive.getCurrentPosition());
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("ServoLeft Position", leftServoPosition);
        telemetry.addData("ServoRight Position", rightServoPosition);
        telemetry.addData("CRSLeft Power", intakeCRSLeft.getPower());
        telemetry.addData("CRSRight Power", intakeCRSRight.getPower());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        packet.put("x", drive.pose.position.x);
        packet.put("y", drive.pose.position.y);
        packet.put("heading", Math.toDegrees(drive.pose.heading.toDouble()));
        packet.put("intakePosition", intakeDrive.getCurrentPosition());
        packet.put("P", P);
        packet.put("I", I);
        packet.put("D", D);
        packet.put("F", F);
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        dashboard.sendTelemetryPacket(packet);
    }
}
