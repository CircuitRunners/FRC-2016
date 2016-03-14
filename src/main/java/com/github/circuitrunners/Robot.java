package com.github.circuitrunners;

import com.github.circuitrunners.akilib.SmartDashboard2;
import com.github.circuitrunners.calib.CalibMath;
import com.github.circuitrunners.commands.ControlDrive;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class Robot extends IterativeRobot {

    // Camera Constants
    private static final double CAMERA_RESOLUTION_Y = 480;
    private static final double CAMERA_RESOLUTION_X = 640;
    private static final double TOP_TARGET_HEIGHT = 97;
    private static final double VERTICAL_FOV = 51.4;
    private static final double HORIZONTAL_FOV = 67;
    private static final double CAMERA_ANGLE = 13;
    private static final double TOP_CAMERA_HEIGHT = 14;
    
    private String weatherStatus;

    private final NetworkTable grip = NetworkTable.getTable("GRIP");

    private ExecutorService sequentialExecutor = Executors.newSingleThreadExecutor();
    private ExecutorService parallelExecutor = Executors.newCachedThreadPool();

    private static OI oi;

    @Override
    public void robotInit() {
        RobotMap.liftMotor.setEncPosition(0);
        NetworkTable.setIPAddress("10.10.2.2");
        NetworkTable.globalDeleteAll();
//        RobotMap.camera = new AxisCamera("10.10.2.11");
    }

    @Override
    public void autonomousInit() {
        sequentialExecutor.execute(new AutonomousDriveThread(1, 0, 1000));
        sequentialExecutor.execute(new AutonomousDriveThread(-1, 0, 1000));
    }

    private class AutonomousDriveThread implements Runnable {
        double moveVal;
        double rotateVal;
        int waitTime;

        public AutonomousDriveThread(double moveVal, double rotateVal) {
            this(moveVal, rotateVal, 0);
        }

        public AutonomousDriveThread(double moveVal, double rotateVal, int waitTime) {
            this.moveVal = moveVal;
            this.rotateVal = rotateVal;
            this.waitTime = waitTime;
        }

        @Override
        public void run() {
            RobotMap.robotDrive.arcadeDrive(moveVal, rotateVal);
            if (waitTime > 0) {
                try {
                    Thread.sleep(waitTime);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        oi = new OI();
        RobotMap.liftMotor.setEncPosition(0);
        weatherStatus = CalibMath.answerQuestion();
        Scheduler.getInstance().add(new ControlDrive());
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();

        // Gyro reset
        if (oi.buttonGyroReset.get()) RobotMap.imu.reset();

        // Debug

        //Weather
        SmartDashboard2.put("Temperature", RobotMap.imu.getTemperature());
        SmartDashboard2.put("Pressure", RobotMap.imu.getBarometricPressure());
        SmartDashboard2.put("Will it rain?", weatherStatus);

        SmartDashboard2.put("distanceTraveledX", RobotMap.imu.getMagX());
        SmartDashboard2.put("distanceTraveledY", RobotMap.imu.getMagY());
        SmartDashboard2.put("distanceTraveledZ", RobotMap.imu.getMagZ());

        double[] temp = getStuff();
        if (temp != null) {
            SmartDashboard2.put("distance", temp[0]);
            SmartDashboard2.put("azimuth", temp[1]);
        }
    }

    public double[] getStuff(){
        try {
            final ITable table = grip.getSubTable("contours");

            int largest = CalibMath.getLargestIndex(table.getNumberArray("area", new double[0]));

            double[] centerXs = table.getNumberArray("centerX", new double[0]);
            double centerX = centerXs[largest];

            double[] centerYs = table.getNumberArray("centerY", new double[0]);
            double centerY = centerYs[largest];

            double[] widths = table.getNumberArray("width", new double[0]);
            double width = widths[largest];

            double[] heights = table.getNumberArray("height", new double[0]);
            double height = heights[largest];

            double[] areas = table.getNumberArray("area", new double[0]);
            double area = areas[largest];

            double y = -((2 * (centerY / CAMERA_RESOLUTION_Y)) - 1);
            double distance = (TOP_TARGET_HEIGHT - TOP_CAMERA_HEIGHT) /
                    Math.tan((y * VERTICAL_FOV / 2.0 + CAMERA_ANGLE) * Math.PI / 180);
            //				angle to target...would not rely on this
            double targetX = (2 * (centerX / CAMERA_RESOLUTION_X)) - 1;
            double azimuth = CalibMath.normalize360(targetX * HORIZONTAL_FOV / 2.0 + 0);

            double[] values = {distance, azimuth};
            return values;
        } catch (ArrayIndexOutOfBoundsException e) {
            return new double[]{0, 0};
        }
    }

    @Override
    public void disabledInit() {
        System.out.println("Not default IterativeRobot.disabledInit() method... Overload me, Chandler!");
    }
}
