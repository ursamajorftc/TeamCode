/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="deposit testing", group="Linear OpMode")

public class ArmTest extends LinearOpMode {

    // Declare OpMode members.

    
    private ElapsedTime runtime = new ElapsedTime();
    private Servo clawServo = null;
    private Servo wristServo = null;
    private Servo armServo = null;


    private DcMotor outmoto1 = null;
    private DcMotor outmoto2 = null;
    private static final int MIN_POSITION = 0;
    private static final int MAX_POSITION = 3600;
    private boolean previousAState = false;
    private boolean previousYState = false;

    public double ServoPosition = 0.5;

    //wrist positions
    public double wristPositionDown = 0;
    public double wristPositionStraight = 0.62;
    public double wristPositionOut = 1;

    //arm positions
    public double armPositionDeposit = 0;
    public double armPositionHover = 0.64;
    public double armPositionGrab = 0.7;

    //claw positions
    public double clawPositionOpen = 0.26;
    public double clawPositionClosed = 0.48;

    int depositTargetPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        outmoto1 = hardwareMap.get(DcMotor.class, "outmoto1");
        outmoto2 = hardwareMap.get(DcMotor.class, "outmoto2");

        outmoto1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outmoto1.setTargetPosition(0);
        outmoto1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outmoto1.setPower(0.1);
        

        // To drive forward, most robots need the outmoto1 on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Wait for the game to start (driver presses START)
        waitForStart();

       // Thread armControl = new Thread(new ArmTest.armController());
       // Thread depositSlideControl = new Thread(new ArmTest.slideController());

       // armControl.start();
       // depositSlideControl.start();

        runtime.reset();
        armServo.setPosition(0.15);
        wristServo.setPosition(wristPositionDown);
        clawServo.setPosition(clawPositionClosed);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


//            outmoto1.setPower(-gamepad1.right_stick_y * 1);
//            outmoto2.setPower(gamepad1.right_stick_y * 1);



//             Setup a variable for each drive wheel to save power level for telemetry



          if (gamepad1.y && !previousYState){
              outmoto1.setTargetPosition(3600);
              outmoto1.setPower(1);
              outmoto2.setPower(-outmoto1.getPower());


            }
          if (gamepad1.a && !previousAState){
              outmoto1.setTargetPosition(0);
              outmoto1.setPower(1);
              outmoto2.setPower(0);
              telemetry.addData("yippee", gamepad1.a);

          }

          if (!outmoto1.isBusy()) {
              outmoto2.setPower(0);

          }



            //clawServo.setPosition(ServoPosition);
            previousAState = gamepad1.a;
            previousYState = gamepad1.y;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("IntakeServoPosition", ServoPosition);
            telemetry.addData("IntakePosition", outmoto1.getCurrentPosition());
            telemetry.addData("outmoto1 Power", outmoto1.getPower());
            telemetry.addData("outmoto2 Power", outmoto2.getPower());
            telemetry.addData("Target Position", outmoto1.getTargetPosition());




            telemetry.update();
        }
    }

    private class slideController implements Runnable {
        @Override
        public void run() {
            while (opModeIsActive()) {
                double joystickInput = -gamepad1.right_stick_y;
                int depositPosition = outmoto1.getCurrentPosition();

                depositTargetPosition = depositTargetPosition + (int) (joystickInput * 20);

                if (depositTargetPosition < MIN_POSITION) {
                    depositTargetPosition = MIN_POSITION;
                } else if (depositTargetPosition > MAX_POSITION) {
                    depositTargetPosition = MAX_POSITION;

                }

                outmoto1.setTargetPosition(depositTargetPosition);
                outmoto1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outmoto1.setPower(1);
                outmoto2.setPower(-outmoto1.getPower());

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
