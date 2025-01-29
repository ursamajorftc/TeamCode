package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.locks.Lock;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "guh", group = "Linear OpMode")
@Config
@Disabled
public class ProdTeleOP extends LinearOpMode {

    private DcMotor intakeDrive;
    private DcMotor outtakeDrive1;
    private DcMotor outtakeDrive2;

    private Servo intakeServoLeft;
    private Servo intakeServoRight;
    private CRServo intakeCRSLeft;
    private CRServo intakeCRSRight;
    private Servo armServo;
    private Servo clawServo;

    private Servo wristServo;
    private Servo lockServo;
    private AnalogInput AxonServo1;
    private AnalogInput AxonServo2;

    @Override

    public void resetRuntime() {
        super.resetRuntime();
    }


    public static double ARMGRAB = -0.3; // arm at 90 degrees parallel with the intake
    public static double WRIST90 = 0.5; // pos for grabby grabby block
    public static double FINGEROPEN = 0.4; // pos for open finger
    public static double FINGERCLOSE = 0; // pos for close finger
    public static int ARMVERTICAL = 0; // VERTICAL WITH RAILS
    //raise the rails
    public static double WRISTSCORE = -0.5; // funny angle for the drop
    // and then finger opens
    public static double VERTDOWNPOWER1 = 0.5;
    public static double VERTDOWNPOWER2 = -0.5;
    public static double VERTDOWNGO1 = 0;
    // these should be default i think
    public static double VERTDOWNGO2 = 0;

    public static int FULL_EXTENSION = 965;
    public static int HALF_EXTENSION = 482;
    public static int QUARTER_EXTENSION = 241;
    public static double INTAKE_DOWN_LPOSITION = 0.58;
    public static double INTAKE_DOWN_RPOSITION = 0.4 ;
    public static double INTAKE_UP_LPOSITION = 0.18;
    public static double INTAKE_UP_RPOSITION = 0.8;
    public static double INTAKE_SPIN_POWER = 1.0;
    public static double UNLOCK = 0;

    public static double LOCK = -0.5;



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
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        lockServo = hardwareMap.get(Servo.class, "lockServo");
        outtakeDrive1 = hardwareMap.get(DcMotor.class, "outmoto1");
        outtakeDrive2 = hardwareMap.get(DcMotor.class, "outmoto2");
        AxonServo1 = hardwareMap.get(AnalogInput.class, "axonServo1");
        AxonServo2 = hardwareMap.get(AnalogInput.class,"axonServo2");
        // Distance sensor object
        // DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Config outmoto1
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // DB - Check this line
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wait();

        while (opModeIsActive()) {
            telemetry.addData("Vert Motor 1 Position", outtakeDrive1.getCurrentPosition());
            telemetry.addData("Vert Motor 1 Power", outtakeDrive1.getPower());
            telemetry.addData("Vert Motor 2 Power", outtakeDrive2.getPower());
            telemetry.update();
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));
            if (gamepad1.right_trigger > 0.5) {
                moveSlideToPosition(FULL_EXTENSION);
                spinIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);
            }
            if (gamepad1.right_bumper) {
                moveSlideToPosition(HALF_EXTENSION);
                spinIntake();
                moveIntakeDown();
                lockServo.setPosition(LOCK);

            }
            if (gamepad1.left_trigger > 0.5) {
                retractSlide();
                lockServo.setPosition(UNLOCK);
                spinIntake();


            }
            while (gamepad1.y && opModeIsActive()) {
                Backspin();
            }
//            if (gamepad1.b) {
//                moveVertSlidesToPos(3500, 1);
//
//
//            }

          //  double distanceInMM = distanceSensor.getDistance(DistanceUnit.MM);

            // Print the distance value (for debugging)
//            telemetry.addData("Distance", distanceInMM);
//            telemetry.getPower();

//            // Check if the distance is less than or equal to 12mm
//            if (distanceInMM <= DISTANCE_THRESHOLD_MM) {
//                setWristDown();
//                setArmDown();
//                sleep(1000);
//                closeClaw();
//                setArmBack();
//                setWristBack();
//            }


            telemetry.addData("Slide Position", intakeDrive.getCurrentPosition());
            intakeDrive.setPower(0);
            outtakeDrive1.setPower(0);
            outtakeDrive2.setPower(0);
            intakeCRSLeft.setPower(0);
            intakeCRSRight.setPower(0);
            telemetry.addData("Status", "Stopped");
            telemetry.update();
        }
    }

    private void moveSlideToPosition(int position) {
        intakeDrive.setTargetPosition(position);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(1.0);

        while (intakeDrive.isBusy() && opModeIsActive()) { // Add opModeIsActive() check
            telemetry.addData("I want to move to:", position);
            telemetry.addData("I am at:", intakeDrive.getCurrentPosition());
            telemetry.update();
        }

        intakeDrive.setPower(0.0); // Stop outmoto1 after reaching the target
    }
    public void moveVertSlidesToPos(int targetPosition, double maxPower) {
        clawServo.setPosition(FINGERCLOSE);
        armServo.setPosition(ARMVERTICAL);
        outtakeDrive1.setTargetPosition(targetPosition);
        outtakeDrive2.setTargetPosition(-targetPosition);

        outtakeDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeDrive1.setPower(maxPower);
        outtakeDrive2.setPower(maxPower);

        while (outtakeDrive1.isBusy() && outtakeDrive2.isBusy() && opModeIsActive()) {
            telemetry.addData("Motor 1", outtakeDrive1.getCurrentPosition());
            telemetry.addData("Motor 2", outtakeDrive2.getCurrentPosition());
            telemetry.update();
        }
        wristServo.setPosition(WRISTSCORE);

        clawServo.setPosition(FINGEROPEN);
        clawServo.setPosition(FINGERCLOSE);
        wristServo.setPosition(WRIST90);
        outtakeDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeDrive1.setTargetPosition(targetPosition);
        outtakeDrive2.setTargetPosition(-targetPosition);
        outtakeDrive1.setPower(VERTDOWNPOWER1);
        outtakeDrive2.setPower(VERTDOWNPOWER2);
        armServo.setPosition(ARMGRAB);



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
//       intakeServoRight.setPosition(INTAKE_UP_RPOSITION);
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
