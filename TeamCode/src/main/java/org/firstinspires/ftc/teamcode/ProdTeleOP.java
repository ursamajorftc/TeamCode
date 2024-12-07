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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "EXPDASHTeleOp", group = "Linear OpMode")
@Config
public class ProdTeleOP extends LinearOpMode {

    private DcMotor intakeDrive;
    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    private Servo clawServo;
    private Servo wristServo;
    private Servo armServo;
    private static final double DISTANCE_THRESHOLD_MM = 12.0;  // Threshold in mm
    public static int FULL_EXTENSION = 965;
    public static int HALF_EXTENSION = 482;
    public static int QUARTER_EXTENSION = 241;
    public static double INTAKE_DOWN_LPOSITION = 0.58;
    public static double INTAKE_DOWN_RPOSITION = 0.4 ;
    public static double INTAKE_UP_LPOSITION = 0.18;
    public static double INTAKE_UP_RPOSITION = 0.8;
    public static double INTAKE_SPIN_POWER = 1.0;
    public static double CLAW_CLOSE = 0.0;
    public static double CLAW_OPEN = 0.4;
    public static double WRIST_DOWN = 0.0;
    public static double WRIST_BACK = 1.0;
    public static double ARM_DOWN = 0.0;
    public static double ARM_BACK = 1.0;

    // end of exposed variables
    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map
        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        // Distance sensor object
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");


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
                spinIntake();
                moveIntakeDown();
            }
            if (gamepad1.right_bumper) {
                moveSlideToPosition(HALF_EXTENSION);
                spinIntake();
                moveIntakeDown();
            }
            if (gamepad1.left_trigger > 0.5) {
                retractSlide();
            }
            while (gamepad1.y) {
                Backspin();
            }
            if (gamepad1.a) {
                clawServo.setPosition(CLAW_OPEN);
            }
            if (gamepad1.b) {
                clawServo.setPosition(CLAW_CLOSE);
            }
            double distanceInMM = distanceSensor.getDistance(DistanceUnit.MM);

            // Print the distance value (for debugging)
            telemetry.addData("Distance", distanceInMM);
            telemetry.update();

            // Check if the distance is less than or equal to 12mm
            if (distanceInMM <= DISTANCE_THRESHOLD_MM) {
                setWristDown();
                setArmDown();
                sleep(1000);
                closeClaw();
                setArmBack();
                setWristBack();
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
//    private void moveIntakeUp() {
//       intakeServoLeft.setPosition(INTAKE_UP_LPOSITION);
//        intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
//    }

    private void spinIntake() {
        intakeCRSLeft.setPower(-INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(INTAKE_SPIN_POWER); // reverse this so we don't explode servos lol
    }
    private void Backspin() {
        intakeCRSLeft.setPower(INTAKE_SPIN_POWER);
        intakeCRSRight.setPower(-INTAKE_SPIN_POWER); // reverse this so we don't explode servos lol
    }
    private void stopIntake() {
        intakeCRSLeft.setPower(0);
        intakeCRSRight.setPower(0);
    }
    private void closeClaw() {
        clawServo.setPosition(CLAW_CLOSE);
    }
    private void openClaw(){
        clawServo.setPosition(CLAW_OPEN);
    }
    private void setWristDown() {
        wristServo.setPosition(WRIST_DOWN);
    }
    private void setWristBack(){
        wristServo.setPosition(WRIST_BACK);
    }
    private void setArmDown(){
        armServo.setPosition(ARM_DOWN);
    }
    private void setArmBack(){
        armServo.setPosition(ARM_BACK);
    }
}
