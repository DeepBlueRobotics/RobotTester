// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.MotorControllerFactory;

public class RobotContainer {

    public final Joystick lJoy = new Joystick(0);
    public final Joystick rJoy = new Joystick(1);
    public final PowerDistribution pdp = new PowerDistribution();
    public final ArrayList<Integer> monitoredPDPPorts = new ArrayList<>();
    public final ArrayList<GyroPort> monitoredGyros = new ArrayList<>();
    public final ArrayList<Integer> monitoredMotorPorts = new ArrayList<>();
    public final ArrayList<Integer> monitoredDIOPorts = new ArrayList<>();
    public final ArrayList<Integer> monitoredSolonoidPorts = new ArrayList<>();

    public static final String PDP_MONITOR_PARAM = "PDP Port to Monitor: ";
    public static final String MOTOR_MONITOR_PARAM = "Motor CAN Port to Connect: ";
    public static final String ENCODER_MONITOR_PARAM_1 = "Encoder DIO Port to Connect (1): ";
    public static final String ENCODER_MONITOR_PARAM_2 = "Encoder DIO Port to Connect (2): ";
    public static final String SOLONOID_MONITOR_PARAM_SINGLE = "Solonoid Port to Connect: ";
    public static final String SOLONOID_MONITOR_PARAM_DOUBLE_1 = "Double Solenoid Port to Connect (1): ";
    public static final String SOLONOID_MONITOR_PARAM_DOUBLE_2 = "Double Solenoid Port to Connect (2): ";

    @SuppressWarnings("unchecked")
    public RobotContainer() {
        new Button("Restart Robot Code", () -> {
            throw new RuntimeException("THIS IS A USER-TRIGGERED EVENT TO RESTART ROBOT CODE!!!");
        });

        SmartDashboard.putNumber(PDP_MONITOR_PARAM, 0);
        new Button("Track PDP Port Current", () -> {
            int port = (int) SmartDashboard.getNumber(PDP_MONITOR_PARAM, 0);
            if (monitoredPDPPorts.contains(port))
                return;
            monitoredPDPPorts.add(port);
            Robot
                    .registerPeriodic(() -> SmartDashboard.putNumber("PDP Current Port " + port, pdp.getCurrent(port)));
        });

        new EnumButton<GyroPort>("Track Gyroscope", GyroPort.class, type -> {
            if (monitoredGyros.contains(type))
                return;
            monitoredGyros.add(type);
            SerialPort.Port port;
            switch (type) {
                case MXP:
                    port = SerialPort.Port.kMXP;
                    break;
                case ONBOARD:
                    port = SerialPort.Port.kOnboard;
                    break;
                case USB:
                    port = SerialPort.Port.kUSB;
                    break;
                case USB1:
                    port = SerialPort.Port.kUSB1;
                    break;
                case USB2:
                    port = SerialPort.Port.kUSB2;
                    break;
                default:
                    port = null;
                    break;
            }
            if (port == null)
                return;
            SmartDashboard.putData("Gyro (" + type.toString() + ")", new AHRS(port));
        });

        SmartDashboard.putNumber(MOTOR_MONITOR_PARAM, 0);
        new EnumButton<>("Connect Motor", MotorType.class, type -> {
            int port = (int) SmartDashboard.getNumber(MOTOR_MONITOR_PARAM, 0);
            if (monitoredMotorPorts.contains(port))
                return;
            monitoredMotorPorts.add(port);
            Consumer<Double> speedSetter = speed -> {
            };
            switch (type) {
                case SPARK_MAX:
                    CANSparkMax spark = MotorControllerFactory.createSparkMax(port);
                    SmartDashboard.putBoolean("SPARK_MAX " + port + " Coast", false);
                    Robot.registerPeriodic(() -> spark.setIdleMode(
                            SmartDashboard.getBoolean("SPARK_MAX " + port + " Coast", false) ? IdleMode.kCoast
                                    : IdleMode.kBrake));
                    speedSetter = spark::set;
                    RelativeEncoder encoder = spark.getEncoder();
                    doubleSetter("SPARK_MAX Encoder", "" + port, "Position", encoder::getPosition);
                    doubleSetter("SPARK_MAX Encoder", "" + port, "Speed", encoder::getVelocity);
                    break;
                case TALON_SRX:
                    WPI_TalonSRX talon = MotorControllerFactory.createTalon(port);
                    SmartDashboard.putBoolean("TALON_SRX " + port + " Coast", false);
                    Robot.registerPeriodic(() -> talon.setNeutralMode(
                            SmartDashboard.getBoolean("TALON_SRX " + port + " Coast", false) ? NeutralMode.Coast
                                    : NeutralMode.Brake));
                    speedSetter = talon::set;
                    break;
                case VICTOR_SPX:
                    WPI_VictorSPX victor = MotorControllerFactory.createVictor(port);
                    SmartDashboard.putBoolean("VICTOR_SPX " + port + " Coast", false);
                    Robot.registerPeriodic(() -> victor.setNeutralMode(
                            SmartDashboard.getBoolean("VICTOR_SPX " + port + " Coast", false) ? NeutralMode.Coast
                                    : NeutralMode.Brake));
                    speedSetter = victor::set;
                    break;
                default:
                    break;
            }
            String speedName = formatName(type.toString(), "" + port, "Speed");
            Supplier<Double> speedGetter = () -> SmartDashboard.getNumber(speedName, 0);;
            SmartDashboard.putNumber(speedName, 0);
            Supplier<Double>[] speedSupplier = new Supplier[] { speedGetter };
            new EnumButton<>(formatName(type.toString(), "" + port, "Control Type"), ControlType.class, "", formatName(type.toString(), "" + port, "Control Type") + ": Change", ctrlType -> {
                switch (ctrlType) {
                    case LJOYX:
                        speedSupplier[0] = lJoy::getX;
                        break;
                    case LJOYY:
                        speedSupplier[0] = lJoy::getY;
                        break;
                    case RJOYX:
                        speedSupplier[0] = rJoy::getX;
                        break;
                    case RJOYY:
                        speedSupplier[0] = rJoy::getY;
                        break;
                    case VALUE:
                        speedSupplier[0] = speedGetter;
                        break;
                    default:
                        break;
                }
            });
            Consumer<Double> finalSpeedSetter = speedSetter;
            Robot.registerPeriodic(() -> finalSpeedSetter.accept(speedSupplier[0].get()));
        });

        SmartDashboard.putNumber(ENCODER_MONITOR_PARAM_1, 0);
        SmartDashboard.putNumber(ENCODER_MONITOR_PARAM_2, 0);
        new Button("Connect Encoder", () -> {
            int port1 = (int) SmartDashboard.getNumber(ENCODER_MONITOR_PARAM_1, 0);
            int port2 = (int) SmartDashboard.getNumber(ENCODER_MONITOR_PARAM_2, 0);
            if(monitoredDIOPorts.contains(port1) || monitoredDIOPorts.contains(port2)) return;
            monitoredDIOPorts.add(port1);
            monitoredDIOPorts.add(port2);

            SmartDashboard.putData(formatName("Encoder", port1 + ", " + port2, ""), new Encoder(port1, port2));
        });

        SmartDashboard.putNumber(SOLONOID_MONITOR_PARAM_DOUBLE_1, 0);
        SmartDashboard.putNumber(SOLONOID_MONITOR_PARAM_DOUBLE_2, 0);
        new Button("Connect Double Solonoid", () -> {
            int port1 = (int) SmartDashboard.getNumber(SOLONOID_MONITOR_PARAM_DOUBLE_1, 0);
            int port2 = (int) SmartDashboard.getNumber(SOLONOID_MONITOR_PARAM_DOUBLE_2, 0);
            if(monitoredSolonoidPorts.contains(port1) || monitoredSolonoidPorts.contains(port2)) return;
            monitoredSolonoidPorts.add(port1);
            monitoredSolonoidPorts.add(port2);

            SmartDashboard.putData(formatName("Double Solenoid", port1 + ", " + port2, ""), new DoubleSolenoid(PneumaticsModuleType.CTREPCM, port1, port2));
        });

        SmartDashboard.putNumber(SOLONOID_MONITOR_PARAM_SINGLE, 0);
        SmartDashboard.putNumber(SOLONOID_MONITOR_PARAM_DOUBLE_2, 0);
        new Button("Connect Double Solonoid", () -> {
            int port = (int) SmartDashboard.getNumber(SOLONOID_MONITOR_PARAM_SINGLE, 0);
            if(monitoredSolonoidPorts.contains(port)) return;
            monitoredSolonoidPorts.add(port);

            SmartDashboard.putData(formatName("Solenoid", port + "", ""), new Solenoid(PneumaticsModuleType.CTREPCM, port));
        });
    }

    public static void doubleGetter(String type, String id, String function, double defaultValue,
            Consumer<Double> func) {
        String name = formatName(type, id, function);

        SmartDashboard.putNumber(name, defaultValue);
        Robot.registerPeriodic(() -> func.accept(SmartDashboard.getNumber(name, defaultValue)));
    }

    public static void doubleSetter(String type, String id, String function, Supplier<Double> func) {
        String name = formatName(type, id, function);

        Robot.registerPeriodic(() -> SmartDashboard.putNumber(name, func.get()));
    }

    public static String formatName(String type, String id, String function) {
        return function.isBlank() ? String.format("%s (%s)", type, id) : String.format("%s (%s): %s", type, id, function);
    }

    public static enum GyroPort {
        MXP, ONBOARD, USB, USB1, USB2;
    }

    public static enum MotorType {
        SPARK_MAX, TALON_SRX, VICTOR_SPX;
    }

    public static enum ControlType {
        VALUE, LJOYX, LJOYY, RJOYX, RJOYY;
    }

}
