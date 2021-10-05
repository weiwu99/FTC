import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by pliu on 1/16/18.
 */

@Autonomous(name = "coreAuto4100", group = "4100")
//@Disabled
public abstract class coreAuto4100 extends LinearOpMode {
    private DcMotor LeftFront, RightFront, LeftBack, RightBack;
//    private DcMotor Lifting;
//    private DcMotor leftTread, rightTread;
//    private DcMotor tread;

    private Servo plate;
    private Servo bottom, top;

    private ColorSensor sensorColor;

    private double initHeading;

    private BNO055IMU imu;

    private Orientation angles;


    public void runSuperMeta(int position, int key) {

        boolean Red = getSide(position);

        hardwareInitialize();

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //Start
        if(opModeIsActive()){

            colorR(Red);

            //Motion
            initHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            composeTelemetry();
            telemetry.update();
            runMeta(position, key);
        }
    }

    void hardwareInitialize(){

        LeftFront = hardwareMap.dcMotor.get("frontleft");
        RightFront = hardwareMap.dcMotor.get("frontright");
        LeftBack = hardwareMap.dcMotor.get("backleft");
        RightBack = hardwareMap.dcMotor.get("rightback");

        plate = hardwareMap.servo.get("glyphFlip");

        top = hardwareMap.servo.get("ballDrop");
        bottom = hardwareMap.servo.get("ballTurn");

    }

    public void runMeta (int posi, int key) {

        switch (posi) {
            case 1:
                movement(0.2, 1500, "backward");

                if(key == 0) movement(0.4, 650, "right");
                if(key == 2) movement(0.4, 200, "left");

                if(key == 0) {
                    turnCheck(35);

                }
                else if(key == 1) {
                    turnCheck(36.5);
                    movement(0.2, 350, "backward");
                }
                else if(key == 2) {
                    turnCheck(16);
                    movement(0.2, 150, "backward");
                }
                else {
                    turnCheck(36.5);
                    movement(0.2, 350, "backward");
                }

                plate("dump");
                sleep(500);

                movement(0.2, 800, "backward");
                movement(0.2, 450, "forward");

                break;

            case 2:
                if(key == 0) {
                    movement(0.2, 3000, "backward");
                    turnCheck(-112);
                    movement(0.4, 300, "right");
                }
                else if(key == 1) {
                    movement(0.2, 2600, "backward");
                    turnCheck(-110);
                }
                else if(key == 2) {
                    movement(0.2, 2200, "backward");
                    turnCheck(-112);
                    movement(0.4, 300, "left");
                }
                else {
                    movement(0.2, 2600, "backward");
                    turnCheck(-110);
                }

                plate("dump");
                sleep(500);

                movement(0.2, 800, "backward");
                movement(0.2, 450, "forward");

                break;

            case 3:
                if(key == 0) {
                    movement(0.2, 2200, "backward");
                    turnCheck(-68);
                    movement(0.4, 300, "right");
                }
                else if(key == 1) {
                    movement(0.2, 2600, "backward");
                    turnCheck(-70);
                }
                else if(key == 2) {
                    movement(0.2, 3000, "backward");
                    turnCheck(-68);
                    movement(0.4, 300, "left");
                }
                else {
                    movement(0.2, 2600, "backward");
                    turnCheck(-70);
                }

                plate("dump");
                sleep(500);

                movement(0.2, 800, "backward");
                movement(0.2, 450, "forward");

                break;

            case 4:
                movement(0.2, 1500, "forward");

                if(key == 0) movement(0.4, 200, "left");
                if(key == 2) movement(0.4, 650, "right");

                if(key == 0) {
                    turnCheck(164);
                    movement(0.2, 150, "backward");
                }
                else if(key == 1) {
                    turnCheck(140);
                    movement(0.2, 350, "backward");
                }
                else if(key == 2) {
                    turnCheck(145);
                    movement(0.2, 550, "backward");
                }
                else {
                    turnCheck(143.5);
                    movement(0.2, 350, "backward");
                }

                plate("dump");
                sleep(500);

                movement(0.2, 800, "backward");
                movement(0.2, 450, "forward");

                break;
        }
    }

//    void Tread(double pwr, int time, String direction) {
//        switch (direction) {
//            case "in":
//                leftTread.setPower(-pwr);
//                rightTread.setPower(pwr);
//                break;
//            case "out":
//                leftTread.setPower(pwr);
//                rightTread.setPower(-pwr);
//                break;
//        }
//
//        sleep(time);
//        leftTread.setPower(0);
//        rightTread.setPower(0);
//    }

    void plate(String status){
        switch(status){
            case "set":
                plate.setPosition(0);
                sleep(800);
                break;
            case "dump":
                plate.setPosition(0.2);
                sleep(500);
                plate.setPosition(0.4);
                sleep(500);
                plate.setPosition(0.6);
                sleep(500);
                plate.setPosition(0.8);
                sleep(500);
                break;
        }
    }

    boolean getSide(int index){
        if(index == 1 || index == 2) return true;
        return false;
    }

    boolean colorDetect(){
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorcd");

        boolean isRed = false;

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        sleep(1000);

        while(opModeIsActive()) {

            double red = sensorColor.red();
            double blue = sensorColor.blue();

            if ( red < blue || Math.abs(red - blue) <= 5) { isRed = false; break;}

            else if (red - blue > 5) { isRed = true; break;}
        }

        return isRed;
    }

    void colorR(boolean red){

        top.setPosition(0.8);
        sleep(300);

        boolean isRed = colorDetect();

        if (red == true) {
            if (isRed == true) {
                bottom.setPosition(0);
                sleep(300);
                bottom.setPosition(0.4);
                sleep(200);

            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.6);
                sleep(200);
            }
        } else {
            if (isRed == false) {
                bottom.setPosition(0);
                sleep(300);
                bottom.setPosition(0.4);
                sleep(200);
            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.6);
                sleep(200);
            }
        }
        top.setPosition(0.2);
        sleep(500);
        bottom.setPosition(0.5);
        sleep(500);

    }

    void turnCheck(double degree){

        double temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double sub = temp;

        if(degree > 0) {
            while (Math.abs(sub - initHeading - degree) > 10) {
                movement(0.4, 200, "turnLeft");
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp < 0)
                    sub = 360 + temp;
            }
        }
        else if (degree < 0) {
            while (Math.abs(sub - initHeading - degree) > 10) {
                movement(0.4, 200, "turnRight");
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0)
                    sub = temp - 360;
            }
        }

        initHeading += degree;

        if(sub - degree > 0) {
            while (Math.abs(sub - degree) > 1 && degree < sub) {
                movement(0.2, 100, "turnRight");
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;

                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
        else if (sub - degree < 0) {
            while (Math.abs(sub - degree) > 1 && degree > sub) {
                movement(0.2, 100, "turnLeft");
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void movement(double pwr, int time, String direction) {
        switch (direction) {
            case "forward":
                LeftFront.setPower(pwr);
                LeftBack.setPower(pwr);
                RightFront.setPower(-pwr);
                RightBack.setPower(-pwr);
                break;
            case "backward":
                LeftFront.setPower(-pwr);
                LeftBack.setPower(-pwr);
                RightFront.setPower(pwr);
                RightBack.setPower(pwr);
                break;
            case "right":
                LeftFront.setPower(-pwr);
                LeftBack.setPower(pwr);
                RightFront.setPower(-pwr);
                RightBack.setPower(pwr);
                break;
            case "left":
                LeftFront.setPower(pwr);
                LeftBack.setPower(-pwr);
                RightFront.setPower(pwr);
                RightBack.setPower(-pwr);
                break;
            case "turnLeft":
                LeftFront.setPower(-pwr);
                LeftBack.setPower(-pwr);
                RightFront.setPower(-pwr);
                RightBack.setPower(-pwr);
                break;
            case "turnRight":
                LeftFront.setPower(pwr);
                LeftBack.setPower(pwr);
                RightFront.setPower(pwr);
                RightBack.setPower(pwr);
                break;
        }

        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

    }

    int getKey(){
        int key = 4;

        String TAG = "Vuforia VuMark Sample";

        String LICENSE = "AcizW6z/////AAAAGUZPTu7RHE/mj7BX78dIYvhHSBQgkIRk8DbyqqV39fgEgeCyLk/inLOzcYFxgjw" +
                "EJ38p0pLEtfKpvNYHGnE8e/sXPl8HKPyiCxTaUeMEuxW//EmlG5KTcGDODGcpO55Kvsn0tARBkSC" +
                "YPUqTmpMMszDP9xgfcRUj/EjX+3eqqLgrLs9DylSDdtdmp6pgInu+oZdUHZuWyvIxl+w9PVxj9cdkBZ" +
                "XRBygWFtD8sduMGSn+mPK6Bm+Dw+PVnLQbDyPyP9M2FFO9NC1h76ADa4E8JmRhBSmottVfzlFpx4/laBYLqAyAha" +
                "TjW+CLmSOItA3e/rJieIleGfmBLNOsp40TJJt4Y7Gz2qpn64qpDH+nQ+xb";

        OpenGLMatrix lastLocation = null;
        VuforiaLocalizer vuforia;

        ElapsedTime runtime = new ElapsedTime();

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameterV = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameterV.vuforiaLicenseKey = LICENSE;

        parameterV.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameterV);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
        runtime.reset();

        while (runtime.seconds() < 4.0) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();

                telemetry.addData("Pose", format(pose));

            } else
                telemetry.addData("VuMark", "not visible");

            telemetry.update();

            if (vuMark.toString() == "LEFT"){ key = 0; break;}

            else if (vuMark.toString() == "CENTER"){ key = 1; break;}

            else if (vuMark.toString() == "RIGHT") { key = 2; break;}

            else break;
        }

        return key;
    }

}