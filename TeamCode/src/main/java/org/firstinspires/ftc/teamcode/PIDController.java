package org.firstinspires.ftc.teamcode;

public class PIDController {

	private double kP;
	private double kI;
	private double kD;

	private double targetPoint;
	private double lastError;
	private double integral;

	public PIDController(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	public double update(double targetPoint, double processVariable) {
		this.targetPoint = targetPoint;

		double error = targetPoint - processVariable;
		integral += error;
		double derivative = error - lastError;
		lastError = error;

		return kP * error + kI * integral + kD * derivative;
	}
}

