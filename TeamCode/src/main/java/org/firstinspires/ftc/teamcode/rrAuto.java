package org.firstinspires.ftc.teamcode;

import android.view.View;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

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
    private static final int MAX_POSITION = 950;
    int intakeTargetPosition = 0;

    public double ServoPosition = 0.5;

    //wrist positions
    public double wristPositionDown = 0;
    public double wristPositionStraight = 0.62;
    public double wristPositionOut = 1;

    //arm positions
    public double armPositionDeposit = 0.1;
    public double armPositionHover = 0.54;
    public double armPositionGrab = 0.675;

    //claw positions
    public double clawPositionOpen = 0.26;
    public double clawPositionClosed = 0.48;

    private boolean previousDpadDownState = false;
    private boolean previousDpadUpState = false;
    private boolean previousAState = false;

    private boolean previousIntakeState = false;
    private Servo intakeServoRight = null;

    boolean sampleDistanceTriggered = false;
    public boolean intakeScoreState = false;
    public boolean downState = false;

    int state = 0; // Persistent state variable
    long startTime = 0; // Persistent timer variable

    NormalizedColorSensor colorSensor;
    NormalizedColorSensor sampleDistance;

    View relativeLayout;

    final float[] hsvValues = new float[3];




    public double intakeServoPosition = 0;
    public Pose2d corner(int angle){
        return new Pose2d(-62, -43, Math.toRadians(angle));
    }
    @Override
    public void runOpMode() throws InterruptedException {
        intakeCRSLeft  = hardwareMap.get(CRServo.class, "intakeCRSLeft");
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
        clawServo.setPosition(clawPositionClosed);
        sleep(250);
        armServo.setPosition(0.15);
        wristServo.setPosition(wristPositionDown);

        Pose2d beginPose = new Pose2d(-33, -62, Math.toRadians(0));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Action trajectoryBucket = drive.actionBuilder(beginPose)
                    .splineToLinearHeading(corner(45), -pi/4)

                    .build();
            Action waitingTrajectory = drive.actionBuilder(corner(45))
                            .waitSeconds(3.5).build();


            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            trajectoryBucket,
                            arm.bottomArmPos(),
                            depositLift.depositUp(),
                            waitingTrajectory,
                            arm.scoreArmPos(),
                            waitingTrajectory,
                            depositLift.depositDown()

                    )
            );

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
        }  else {
            throw new RuntimeException();
        }

        while (opModeIsActive()) {
            if (!outmoto1.isBusy()) {
                outmoto2.setPower(0);
            }
            updateArmRetracty();
            if (intakeScoreState) {
                state = 1;
                telemetry.addData("hello", intakeScoreState);
                telemetry.update();
                wristServo.setPosition(wristPositionOut);
                sleep(250);
                clawServo.setPosition(clawPositionOpen);
                sleep(800);
                clawServo.setPosition(clawPositionClosed);
                intakeScoreState = false;

//               long elapsedTime = System.currentTimeMillis() - startTime;
//
//               switch (state) {
//                   case 1:
//
//                       wristServo.setPosition(wristPositionOut);
//                       state = 2;
//                       telemetry.addData("state", state);
//                       telemetry.update();
//                       startTime = System.currentTimeMillis(); // Reset timer
//                       break;
//
//                   case 2:
//                       if (elapsedTime >= 150) {
//
//                           clawServo.setPosition(clawPositionOpen);
//                           state = 3;
//                           startTime = System.currentTimeMillis(); // Reset timer
//                       }
//                       break;
//                   case 3:
//                       state = 0;
//                       intakeScoreState = false;
//                       break;

//            }


           }



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
  public class ScoreArmPos implements Action {
        @Override
      public boolean run(@NonNull TelemetryPacket packet){
            intakeScoreState = true;
            telemetry.addData("yes", intakeScoreState);
            telemetry.update();
//            wristServo.setPosition(wristPositionOut);
//            clawServo.setPosition(clawPositionOpen);
            return false;
        }


  }
  public Action scoreArmPos(){
        return new ScoreArmPos();
  }

  }
  public class DepositLift {
        public class DepositUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outmoto1.setTargetPosition(3300);
                outmoto2.setPower(1);
                outmoto1.setPower(1);
                return false;
            }
        }
        public Action depositUp() {
            return new DepositUp();
        }

        public class DepsitDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                downState = true;
                clawServo.setPosition(clawPositionClosed);

                return false;
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
                    downState = false;
                    // Transition to CLOSE_CLAW state
                    clawServo.setPosition(clawPositionClosed);
                    stateStartTime = currentTime; // Record the time
                    currentState = mainTeleOp.RobotState.CLOSE_CLAW;
                }
                break;

            case CLOSE_CLAW:
                if (currentTime - stateStartTime >= 1200) { // Wait 200ms
                    armServo.setPosition(0.15);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.MOVE_ARM;
                }
                break;

            case MOVE_ARM:
                if (currentTime - stateStartTime >= 1200) { // Wait 200ms
                    wristServo.setPosition(wristPositionDown);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.MOVE_WRIST;
                }
                break;

            case MOVE_WRIST:
                if (currentTime - stateStartTime >= 1200) { // Wait 200ms
                    clawServo.setPosition(clawPositionOpen);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                if (currentTime - stateStartTime >= 5200) { // Wait 200ms
                    outmoto1.setTargetPosition(0);
                    outmoto1.setPower(1);
                    outmoto2.setPower(0);
                    telemetry.addData("yippee", gamepad1.a);
                    stateStartTime = currentTime;
                    currentState = mainTeleOp.RobotState.COMPLETE;
                }
                break;

            case COMPLETE:
                if (currentTime-stateStartTime > 1200) {
                    currentState = mainTeleOp.RobotState.IDLE;
                }
                // All actions complete; stay idle or transition as needed
                break;
        }
    }



}




