package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "PIDTest", group = "Linear OpMode")
public class PIDTest extends LinearOpMode {
	// motors declaration, we use the Ex version as it has velocity measurements
	DcMotorEx outmoto1;
	DcMotorEx outmoto2;
	PIDController control = new PIDController(0.01,0,0.2);
	@Override
	public void runOpMode() throws InterruptedException {
		// the string is the hardware map name
		outmoto1 = hardwareMap.get(DcMotorEx.class, "outmoto1");
		outmoto2 = hardwareMap.get(DcMotorEx.class, "outmoto2");

		// use braking to slow the motors down faster
		outmoto1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		outmoto2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		// disables the default velocity control
		// this does NOT disable the encoder from counting,
		// but lets us simply send raw motors power.
		outmoto1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		outmoto2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		waitForStart();

		int targetPosition = 2300;

		// loop that runs while the program should run.
		while (opModeIsActive()) {
			double upPower = control.update(targetPosition, outmoto1.getCurrentPosition());
			double downPower = control.update(0, outmoto1.getCurrentPosition());

			if ((gamepad1.dpad_up)) {
				outmoto1.setPower(upPower);
				outmoto2.setPower(-upPower);
			} else if (gamepad1.dpad_down) {
				outmoto2.setPower(0);
				outmoto1.setPower(downPower);
			}



			double processVariable = outmoto1.getCurrentPosition();
			telemetry.addData("Target Position", targetPosition);
			telemetry.addData("Current Position", processVariable);
			telemetry.addData("Error", targetPosition - processVariable);
			telemetry.addData("Integral", control.getIntegral()); // Assuming 'integral' is public or you add a getter
			telemetry.addData("Command (Output)", upPower);
			telemetry.update();
		}
	}
}

