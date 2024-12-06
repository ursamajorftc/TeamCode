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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "EXPDASHTeleOp", group = "Linear OpMode")
@Config
public class ProdTeleOP extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    public static int FULL_EXTENSION = 965;
    public static int HALF_EXTENSION = 482;
    public static int QUARTER_EXTENSION = 241;
    public static double INTAKE_DOWN_LPOSITION = 0.41;
    public static double INTAKE_DOWN_RPOSITION = -0.41;
    public static double INTAKE_UP_LPOSITION = 1.0;
    public static double INTAKE_UP_RPOSITION = 0.0;
    public static double INTAKE_SPIN_POWER = 1.0;
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
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));
            if (gamepad1.right_trigger > 0.5) {
                moveSlideToPosition(FULL_EXTENSION);
                moveIntakeDown();
                spinIntake();
            } else if (gamepad1.right_bumper) {
                moveSlideToPosition(HALF_EXTENSION);
                moveIntakeDown();
                spinIntake();
            } else if (gamepad1.y) {
                moveSlideToPosition(QUARTER_EXTENSION);
                moveIntakeDown();
                spinIntake();
            } else if (gamepad1.x) {
                retractSlide();
            } else if (gamepad1.dpad_up) {

            } else if (gamepad1.dpad_down) {

            }


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
        intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveSlideToPosition(0);
        intakeDrive.setPower(0.0);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    private void moveIntakeDown() {
        // intakeServoLeft.setPosition(INTAKE_DOWN_LPOSITION);
       intakeServoRight.setPosition(INTAKE_DOWN_RPOSITION);
    }
    private void moveIntakeUp() {
        // intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
    }

    private void spinIntake() {
        intakeCRSLeft.setPower(-INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(INTAKE_SPIN_POWER); // reverse this so we don't explode servos lol
    }
    private void stopIntake() {
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
}
