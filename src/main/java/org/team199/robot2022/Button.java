package org.team199.robot2022;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Button implements Runnable {

    public final String name;
    public final Runnable onClick;

    public Button(String name, Runnable onClick) {
        this.name = name;
        this.onClick = onClick;
        SmartDashboard.putBoolean(name, false);
        Robot.registerPeriodic(this);
    }

    @Override
    public void run() {
        if(SmartDashboard.getBoolean(name, false)) {
            SmartDashboard.putBoolean(name, false);
            onClick.run();
        }
    }

}
