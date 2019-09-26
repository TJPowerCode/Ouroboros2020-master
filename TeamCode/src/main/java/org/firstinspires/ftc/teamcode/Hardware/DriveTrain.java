package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {

    private static double motorCounts = 1440;
    private static double gearUp = 1;
    private static double wheelDiam = 4;
    private static double inchCounts = (motorCounts / gearUp) / (wheelDiam * Math.PI);

    public ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode;
    private Sensors sensors;

    public DcMotor fl; //Front Left Motor
    public DcMotor fr; //Front Right Motor
    public DcMotor bl; //Back Left Motor
    public DcMotor br; //Back Right Motor

    public double prevError = 0;
    public double prevTime = 0;
    public double power = 0;
    public double integral;
    public double derive;
    public double proportional;
    public double error;
    public double time;
    public boolean testBoolean;

    public void initDriveTrain(LinearOpMode opMode) {

        this.opMode = opMode;
        sensors = new Sensors();

        //Sets Hardware Map
        fl = this.opMode.hardwareMap.dcMotor.get("fl");
        fr = this.opMode.hardwareMap.dcMotor.get("fr");
        bl = this.opMode.hardwareMap.dcMotor.get("bl");
        br = this.opMode.hardwareMap.dcMotor.get("br");

        //Sets Motor Directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        //Set Power For Static Motors - When Robot Not Moving
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Making 90 Degree Turns
    public void turn(double speed, boolean isRight) {

        if (isRight) {
            fl.setPower(speed);
            fr.setPower(-speed);
            bl.setPower(speed);
            br.setPower(-speed);
        }
        else
        {
            fl.setPower(-speed);
            fr.setPower(speed);
            bl.setPower(-speed);
            br.setPower(speed);
        }
    }

    //Method for Resetting Encoders
    public void resetEncoders() {

        opMode.telemetry.addData("Status", "Resetting Encoders");
        opMode.telemetry.update();

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        opMode.telemetry.addData("Path0", "Starting at %7d : %7d",
                bl.getCurrentPosition(),
                br.getCurrentPosition());
        opMode.telemetry.update();
    }

    public void encoderDrive(double speed,
                             double leftBlinches, double leftInches, double rightInches, double rightBlinches,
                             double timeoutS) {

        double headingTarget = sensors.getGyroYaw();
        double blankTarget = 0;

        int newLeftTarget;
        int newRightTarget;
        int newRightBlarget;
        int newLeftBlarget;

        boolean negVal;

        if (opMode.opModeIsActive()) {
            newLeftTarget = fl.getCurrentPosition() + (int) (leftInches * inchCounts);
            newRightTarget = fr.getCurrentPosition() + (int) (rightInches * inchCounts);
            newLeftBlarget = bl.getCurrentPosition() + (int) (leftBlinches * inchCounts);
            newRightBlarget = br.getCurrentPosition() + (int) (rightBlinches * inchCounts);

            fl.setTargetPosition(newLeftTarget);
            fr.setTargetPosition(newRightTarget);
            bl.setTargetPosition(newLeftBlarget);
            br.setTargetPosition(newRightBlarget);

            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            fl.setPower(Math.abs(speed));
            fr.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));


            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (bl.isBusy() && br.isBusy())) {

                    if (sensors.getGyroYaw() > headingTarget + 0.5 || sensors.getGyroYaw() < headingTarget - 0.5) {
                    blankTarget = headingTarget - sensors.getGyroYaw();
                        if (blankTarget > 0) {
                        blankTarget = blankTarget + 360;
                        }
                        if (blankTarget < sensors.getGyroYaw()) {
                        negVal = true;
                        blankTarget = blankTarget - 180;
                        }
                        else {
                        negVal = false;
                        }
                    turnPID(blankTarget, negVal, 0.6/90, 0.0325, 0.2, 3000);
                    blankTarget = 0;
                    }

                    opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                            fl.getCurrentPosition(),
                            fr.getCurrentPosition());

                    opMode.telemetry.update();

            }

            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);

            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            opMode.sleep(50);
        }

    }

    public void runEncoders() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //double kP = 0.6/90;
    //double kI = 0.0325;
    //double kD = 0.2;

    //PID Turns for Macanum Wheels
    //Proportional Integral Derivative Turn
    public void turnPID (double goal, boolean isRight, double kP, double kI, double kD, double timeOutMS) {

        runtime.reset();
        sensors.angles = sensors.gyro.getAngularOrientation();

        while (opMode.opModeIsActive() && runtime.milliseconds() <= timeOutMS && goal - sensors.getGyroYaw() > 1 ) {

            sensors.angles = sensors.gyro.getAngularOrientation();

            error = goal - sensors.getGyroYaw();
            proportional = error * kP;
            time = runtime.milliseconds();
            integral += ((time - prevTime) * error) * kI;
            derive = ((error - prevError) / (time - prevTime)) * kD;
            power = proportional + integral + derive;

            turn(power, isRight);

            prevTime = runtime.milliseconds();
            prevError = goal - sensors.getGyroYaw();


        }
    }

    public void snowWhite () {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

}
