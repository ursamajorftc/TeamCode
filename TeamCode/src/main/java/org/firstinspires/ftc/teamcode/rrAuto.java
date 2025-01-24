package org.firstinspires.ftc.teamcode;

import android.view.View;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.Objects;

@Config
@Autonomous(name = "rrAuto", group = "Autonomous")
public class rrAuto extends LinearOpMode {

    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intakeCRSLeft = null;
    private CRServo intakeCRSRight = null;
    private Servo intakeServoLeft = null;
    private Servo lockServo = null;
    private DcMotor intakeDrive = null;

    private Servo clawServo = null;
    private Servo wristServo = null;
    private Servo armServo = null;


    private DcMotor outmoto1 = null;
    private DcMotor outmoto2 = null;
    private static final int MIN_POSITION = 0;
    private static final int INTAKE_POSITION1 = 950;
    private static final int INTAKE_POSITION2 = 950;
    private static final int INTAKE_POSITION3 = 950;


    int intakeTargetPosition = 0;

    public double ServoPosition = 0.5;

    //wrist positions
    public double wristPositionDown = 0;
    public double wristPositionStraight = 0.62;
    public double wristPositionOut = 1;

    //arm positions
    public double armPositionDeposit = 0.425;
    public double armPositionHover = 0.815;
    public double armPositionGrab = .95;

    //claw positions
    public double clawPositionOpen = 0.26;
    public double clawPositionClosed = 0.48;

    private boolean previousDpadDownState = false;
    private boolean previousDpadUpState = false;
    private boolean previousAState = false;

    private boolean previousIntakeState = false;
    private boolean intakeComplete = false;
    boolean sampleDistanceTriggered = false;
    boolean intakeJerk = false;

    private Servo intakeServoRight = null;
    private boolean transferComplete = false;

    public boolean intakeScoreState = false;
    public boolean downState = false;

    int state = 0; // Persistent state variable
    long startTime = 0; // Persistent timer variable

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor sampleDistance;

    View relativeLayout;

    final float[] hsvValues = new float[3];

    boolean climax = false;
    public Pose2d corner1(double angle) {
        return new Pose2d(-61, -47, Math.toRadians(angle));
    }

    public Pose2d corner(double angle) {
        return new Pose2d(-63, -46, Math.toRadians(angle));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        intakeCRSLeft = hardwareMap.get(CRServo.class, "intakeCRSLeft");
        intakeCRSRight = hardwareMap.get(CRServo.class, "intakeCRSRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        lockServo = hardwareMap.get(Servo.class, "lockServo");

        intakeDrive = hardwareMap.get(DcMotor.class, "intakeDrive");
        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        outmoto1 = hardwareMap.get(DcMotor.class, "outmoto1");
        outmoto2 = hardwareMap.get(DcMotor.class, "outmoto2");

        outmoto1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outmoto1.setTargetPosition(0);
        outmoto1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outmoto1.setPower(0.1);

        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDrive.setTargetPosition(0);
        intakeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeDrive.setPower(0.5);
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        sampleDistance = hardwareMap.get(NormalizedColorSensor.class, "sampleDistance");

        double pi = Math.PI;
        Arm arm = new Arm();
        DepositLift depositLift = new DepositLift();
        Intake intake = new Intake();
        clawServo.setPosition(clawPositionClosed);
        sleep(250);
        armServo.setPosition(0.525);
        wristServo.setPosition(wristPositionStraight);
        intakeServoLeft.setPosition(0.32);
        intakeServoRight.setPosition(0.695);

        Pose2d beginPose = new Pose2d(-33, -62, Math.toRadians(0));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Action trajectoryBucket = drive.actionBuilder(beginPose)
                    .setTangent(pi)
                    .splineToLinearHeading(corner1(45), pi)

                    .build();
            Action waitingTrajectory = drive.actionBuilder(corner1(42))
                    .waitSeconds(0.5)
                    .build();
            Action waitingTrajectory1 = drive.actionBuilder(corner(45))
                    .waitSeconds(0.5)
                    .build();
            Action waitingTrajectory2 = drive.actionBuilder(corner(45))
                    .waitSeconds(0.5)
                    .build();
            Action waitingTrajectory3 = drive.actionBuilder(new Pose2d(-65, -47, Math. toRadians(45)))
                    .waitSeconds(0.5)
                    .build();
            Action trajectorySample1 = drive.actionBuilder(corner1(45))
//                    .turn(Math.toRadians(22))
                    .splineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(79)), -pi / 8)
                    .build();
            Action trajectoryBucket1 = drive.actionBuilder(new Pose2d(-60, -36, Math.toRadians(79)))
                    .splineToLinearHeading(corner(45), -pi / 4)
                    .build();
            Action trajectorySample2 = drive.actionBuilder(corner(45))
//                    .turn(Math.toRadians(22))
                    .splineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(95)), -pi / 8)
                    .build();
            Action trajectoryBucket2 = drive.actionBuilder(new Pose2d(-60, -36, Math.toRadians(95)))
                    .splineToLinearHeading(new Pose2d(-65, -47, Math.toRadians(45)), -pi / 4)
                    .build();
            Action trajectorySample3 = drive.actionBuilder(new Pose2d(-65, -47, Math.toRadians(45)))
//                    .turn(Math.toRadians(22))
                    .splineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(113)), -pi / 8)
                    .build();
            Action trajectoryBucket3 = drive.actionBuilder(new Pose2d(-60, -36, Math.toRadians(113)))
                    .splineToLinearHeading(new Pose2d(-54, -55, Math.toRadians(35)), -pi / 4)
                    .build();
//            Action waitingTrajectoryPark = drive.actionBuilder(new Pose2d(-54, -55, Math.toRadians(35))
//
//                    .build();


            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(trajectoryBucket,
                                    depositLift.depositUp()),
                            arm.scoreArmPos(),
                            waitingTrajectory,
                            depositLift.depositDown(),
                            trajectorySample1,
                            arm.bottomArmPos(),
                            intake.intakeOut(),
//                            waitingTrajectory,a
                            new ParallelAction(arm.armTransferPos(),
                                    trajectoryBucket1),
                            depositLift.depositUp(),
                            arm.scoreArmPos(),
                            waitingTrajectory1,
                            depositLift.depositDown(),
                            trajectorySample2,
                            arm.bottomArmPos(),
                            intake.intakeOut2(),
//                            waitingTrajectory,
                            new ParallelAction(arm.armTransferPos(),
                                    trajectoryBucket2),
                            depositLift.depositUp(),
                            arm.scoreArmPos(),
                            waitingTrajectory2,
                            depositLift.depositDown(),
                            trajectorySample3,
                            arm.bottomArmPos(),
                            intake.intakeOut1(),
//                            waitingTrajectory,
                            new ParallelAction(arm.armTransferPos(),
                                    trajectoryBucket3),
                            depositLift.depositUp(),
                            arm.scoreArmPos(),
                            waitingTrajectory3,
                            depositLift.depositDown()
//                            park


//                            depositLift.depositUp(),
//                            depositLift.depositDown()

                    )
            );

            intakeCRSLeft.setPower(0);
            intakeCRSRight.setPower(0);

//            Actions.runBlocking(
//                    drive.actionBuilder(new Pose2d(-33, -62, Math.toRadians(0)))
//                            .splineToLinearHeading(corner(45), -pi/4)
//
//                            .waitSeconds(1)

            //code to drop off sample (top bucket)


//                        //face first sample
//                        .splineToLinearHeading(corner(76.79), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //rotate to bucket
//                        .splineToLinearHeading(new Pose2d(corner(45)), -pi/8)
//                        .waitSeconds(1)
//
//                        //drop sample
//
//
//                        //face second sample
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(93.98)), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //rotate to bucket
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
//                        .waitSeconds(1)
//
//                        //drop sample
//
//
//                        //face third sample
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(111.345)), -pi/8)
//                        .waitSeconds(1)
//
//                        //grab
//
//
//                        //face bucket
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
//
//                        //drop sample
//
//
//                        //park
//                        .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(100)), -pi/8)
//                        .splineTo(new Vector2d(-25, -11.5), 0)
            //  .build());
        } else {
            throw new RuntimeException();
        }

        while (opModeIsActive()) {


            if (intakeDrive.getCurrentPosition() < 150) {
                lockServo.setPosition(0.3);
                intakeCRSLeft.setPower(-1);
                intakeCRSRight.setPower(1);
            }

            if (!outmoto1.isBusy()) {
                outmoto2.setPower(0);
            }


//            if (intakeScoreState) {
//                state = 1;
//                telemetry.addData("hello", intakeScoreState);
//                telemetry.update();
//                wristServo.setPosition(wristPositionOut);
//                sleep(250);
//                clawServo.setPosition(clawPositionOpen);
//                sleep(800);
//                clawServo.setPosition(clawPositionClosed);
//                intakeScoreState = false;
//
////               long elapsedTime = System.currentTimeMillis() - startTime;
////
////               switch (state) {
////                   case 1:
////
////                       wristServo.setPosition(wristPositionOut);
////                       state = 2;
////                       telemetry.addData("state", state);
////                       telemetry.update();
////                       startTime = System.currentTimeMillis(); // Reset timer
////                       break;
////
////                   case 2:
////                       if (elapsedTime >= 150) {
////
////                           clawServo.setPosition(clawPositionOpen);
////                           state = 3;
////                           startTime = System.currentTimeMillis(); // Reset timer
////                       }
////                       break;
////                   case 3:
////                       state = 0;
////                       intakeScoreState = false;
////                       break;
//
////            }
//
//
//           }


        }
    }

    public enum IntakeState {
        INTAKE0,
        INTAKE300,
        INTAKEDOWN,
        INTAKE880,
        INTAKESTOP,
        INTAKE350
    }

    public class Intake {
        public class IntakeOut1 implements Action {
            private boolean initialized = false;


            private long nowStartTime = 0;
            double intakeState = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeComplete = false;
                    initialized = true; // Ensure initialization happens only once
                }

                intakeMovement(590);

//                    intakeTargetPosition = 300;
//                    intakeDrive.setPower(1);
//                    sleep(12500);
////
//                    intakeServoLeft.setPosition(0.54);
//                    intakeServoRight.setPosition(0.45);
//                    intakeCRSLeft.setPower(-1);
//                    intakeCRSRight.setPower(1);
//                    lockServo.setPosition(0);
//                    intakeDrive.setPower(0.2);
//                    intakeTargetPosition = 880;
//                    sleep(740);
//
//                    intakeTargetPosition = 0;
//                    intakeDrive.setPower(0.75);
//                    intakeServoLeft.setPosition(0.32);
//                    intakeServoRight.setPosition(0.695);


//
                // Execute one step of intake movement


                // Check if the action is complete


                // Return false to indicate the action is still ongoing

                return !intakeComplete;
            }
        }

        public class IntakeOut2 implements Action {
            private boolean initialized = false;


            private long nowStartTime = 0;
            double intakeState = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeComplete = false;
                    initialized = true; // Ensure initialization happens only once
                }

                intakeMovement(720);

//                    intakeTargetPosition = 300;
//                    intakeDrive.setPower(1);
//                    sleep(12500);
////
//                    intakeServoLeft.setPosition(0.54);
//                    intakeServoRight.setPosition(0.45);
//                    intakeCRSLeft.setPower(-1);
//                    intakeCRSRight.setPower(1);
//                    lockServo.setPosition(0);
//                    intakeDrive.setPower(0.2);
//                    intakeTargetPosition = 880;
//                    sleep(740);
//
//                    intakeTargetPosition = 0;
//                    intakeDrive.setPower(0.75);
//                    intakeServoLeft.setPosition(0.32);
//                    intakeServoRight.setPosition(0.695);


//
                // Execute one step of intake movement


                // Check if the action is complete


                // Return false to indicate the action is still ongoing

                return !intakeComplete;
            }
        }


        public class IntakeOut implements Action {
            private boolean initialized = false;


            private long nowStartTime = 0;
            double intakeState = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeComplete = false;
                    initialized = true; // Ensure initialization happens only once
                }

                intakeMovement(800);

//                    intakeTargetPosition = 300;
//                    intakeDrive.setPower(1);
//                    sleep(12500);
////
//                    intakeServoLeft.setPosition(0.54);
//                    intakeServoRight.setPosition(0.45);
//                    intakeCRSLeft.setPower(-1);
//                    intakeCRSRight.setPower(1);
//                    lockServo.setPosition(0);
//                    intakeDrive.setPower(0.2);
//                    intakeTargetPosition = 880;
//                    sleep(740);
//
//                    intakeTargetPosition = 0;
//                    intakeDrive.setPower(0.75);
//                    intakeServoLeft.setPosition(0.32);
//                    intakeServoRight.setPosition(0.695);


//
                // Execute one step of intake movement


                // Check if the action is complete


                // Return false to indicate the action is still ongoing

                return !intakeComplete;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public Action intakeOut1() {
            return new IntakeOut1();
        }

        public Action intakeOut2() {
            return new IntakeOut2();
        }
    }


    //  public class claw

    public class Arm {

        public class BottomArmPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armServo.setPosition(armPositionDeposit);
                clawServo.setPosition(clawPositionClosed);
                wristServo.setPosition(wristPositionDown);

                return false;


            }
        }

        public Action bottomArmPos() {
            return new BottomArmPos();
        }

        public class ArmTransferPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                transferComplete = false;
                updateArmTransfer();
                return !transferComplete;
            }
        }

        public Action armTransferPos() {
            return new ArmTransferPos();
        }

        public class ScoreArmPos implements Action {
            long scoreTime = 0;
            long scoreStartTime = System.currentTimeMillis();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wristServo.setPosition(wristPositionOut);
                clawServo.setPosition(clawPositionOpen);
                scoreTime = scoreStartTime;


                return (System.currentTimeMillis() - scoreTime) < 500;
            }


        }

        public Action scoreArmPos() {
            return new ScoreArmPos();
        }

    }

    public class DepositLift {
        public class DepositUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outmoto1.setTargetPosition(2300);
                outmoto2.setPower(-1);
                outmoto1.setPower(1);
                if (outmoto1.getCurrentPosition() > 2295) {
                    outmoto2.setPower(0);
                }

                return (outmoto1.getCurrentPosition() <= 2300);
            }
        }

        public Action depositUp() {
            return new DepositUp();
        }

        public class DepsitDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                downState = true;
                updateArmRetracty();


                return downState;
            }

        }

        public Action depositDown() {
            return new DepsitDown();
        }
    }

    public enum RobotState {
        IDLE,
        CLOSE_CLAW,
        MOVE_ARM,
        MOVE_WRIST,
        OPEN_CLAW,
        COMPLETE
    }

    private mainTeleOp.RobotState currentState = mainTeleOp.RobotState.IDLE;
    private long stateStartTime = 0;


    public void updateArmRetracty() {

        // Get the current time in milliseconds
        long currentTime = System.currentTimeMillis();

        switch (currentState) {
            case IDLE:
                if (downState) {
                    // Transition to CLOSE_CLAW state
                    clawServo.setPosition(clawPositionClosed);
                    stateStartTime = currentTime; // Record the time
                    currentState = mainTeleOp.RobotState.CLOSE_CLAW;
                }
                break;

            case CLOSE_CLAW:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    armServo.setPosition(0.475);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.MOVE_ARM;
                }
                break;

            case MOVE_ARM:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    wristServo.setPosition(wristPositionDown);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.MOVE_WRIST;
                }
                break;

            case MOVE_WRIST:
                if (currentTime - stateStartTime >= 200) { // Wait 200ms
                    clawServo.setPosition(clawPositionOpen);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                if (currentTime - stateStartTime >= 100) { // Wait 200ms
                    outmoto1.setTargetPosition(0);
                    outmoto1.setPower(1);
                    outmoto2.setPower(0);
                    telemetry.addData("yippee", gamepad1.a);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.COMPLETE;
                }
                break;

            case COMPLETE:
                if (currentTime - stateStartTime > 200) {
                    currentState = mainTeleOp.RobotState.IDLE;
                    intakeDrive.setTargetPosition(0);
                    downState = false;
                }
                // All actions complete; stay idle or transition as needed
                break;
        }
    }


    private IntakeState intakeState = IntakeState.INTAKE0;
    private long nowStartTime = 0;


    public void intakeMovement(int intakeTargetPosition) {
        if ((intakeDrive.getCurrentPosition() > 145) && outmoto1.getCurrentPosition() < 100) {
            previousIntakeState = true;
            if (outmoto1.getCurrentPosition() < 15) {
                armServo.setPosition(armPositionHover);
                clawServo.setPosition(clawPositionOpen);
            }
        }

        switch (intakeState) {

            case INTAKE0:

                intakeDrive.setTargetPosition(300);
                intakeState = IntakeState.INTAKE300;
                intakeDrive.setPower(1);

                break;

            case INTAKE300:
                if (intakeDrive.getCurrentPosition() > 250) {
                    telemetry.addData("yippee", intakeState);
                    telemetry.update();

                    intakeServoLeft.setPosition(0.54);
                    intakeServoRight.setPosition(0.45);
                    intakeCRSLeft.setPower(-1);
                    intakeCRSRight.setPower(1);
                    lockServo.setPosition(0);
                    intakeState = IntakeState.INTAKEDOWN;
                    intakeDrive.setPower(0.2);
                }
                break;

            case INTAKEDOWN:
                intakeDrive.setTargetPosition(intakeTargetPosition);
                intakeState = IntakeState.INTAKE880;
                nowStartTime = System.currentTimeMillis();

                break;

            case INTAKE880:
                if (intakeDrive.getCurrentPosition() > (intakeTargetPosition - 10)) {
                    intakeDrive.setTargetPosition(0);
                    intakeDrive.setPower(1);
                    intakeServoLeft.setPosition(0.32);
                    intakeServoRight.setPosition(0.695);
                    intakeCRSLeft.setPower(-0.1);
                    intakeCRSRight.setPower(0.1);
                    intakeState = IntakeState.INTAKE0;
                    intakeComplete = true;

                }

                break;


        }
    }

    public void updateArmTransfer() {
        if (intakeDrive.getCurrentPosition() < 150) {
            lockServo.setPosition(0.3);
            intakeCRSLeft.setPower(-0.5);
            intakeCRSRight.setPower(0.5);
        }

        if (!sampleDistanceTriggered && (((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) > 20) && !intakeJerk && intakeDrive.getCurrentPosition() <= 0) {

            intakeDrive.setTargetPosition(200);
            intakeJerk = true;
            climax = true;


        }

        if (intakeJerk && intakeDrive.getCurrentPosition()>90){
            intakeDrive.setTargetPosition(-20);
            intakeJerk = false;

        }


        if (!sampleDistanceTriggered && ((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 20) {
            intakeJerk = false;
            sampleDistanceTriggered = true;
            startTime = System.currentTimeMillis();
            state = 1;
            // Start the state machine
        }


        if (sampleDistanceTriggered) {
            long elapsedTime = System.currentTimeMillis() - startTime;

            switch (state) {
                case 1:
                    armServo.setPosition(armPositionGrab);
                    clawServo.setPosition(clawPositionOpen);
                    wristServo.setPosition(wristPositionDown);
                    state = 2;
                    startTime = System.currentTimeMillis(); // Reset timer
                    break;

                case 2:
                    if (elapsedTime >= 150) {

                        clawServo.setPosition(clawPositionClosed);
                        state = 3;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 3:
                    if (elapsedTime >= 200) {

                        armServo.setPosition(0.475);
                        state = 4;
                        startTime = System.currentTimeMillis(); // Reset timer
                    }
                    break;

                case 4:
                    if (elapsedTime >= 250) {
                        wristServo.setPosition(wristPositionStraight);
                        if (((DistanceSensor) sampleDistance).getDistance(DistanceUnit.MM) < 17) {
                            state = 1;
                        } else {
                            state = 0;
                            sampleDistanceTriggered = false;
                            transferComplete = true;
                        }


                        // End the state machine
                    }
                    break;
            }
        }
    }


}




