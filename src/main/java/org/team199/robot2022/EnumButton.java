package org.team199.robot2022;

import java.util.Arrays;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EnumButton<T extends Enum<T>> {

    public EnumButton(String name, Class<T> options, Consumer<T> onClick) {
        this(name, options, name + ": Type", name + ": Create", onClick);
    }

    @SuppressWarnings({"raw", "rawtypes", "unchecked"})
    public EnumButton(String name, Class<T> options, String typeText, String confirmText, Consumer<T> onClick) {
        SendableChooser chooser = new SendableChooser<>();
        Arrays.stream(options.getEnumConstants()).forEach(option -> chooser.addOption(option.toString(), option));
        SmartDashboard.putData(typeText, chooser);
        new Button(confirmText, () -> {
            onClick.accept((T) chooser.getSelected());
        });
    }

}
