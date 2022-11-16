package com.ma5951.utils.logger;

import java.util.function.Supplier;

public class Value {
    private String valueName;
    private Supplier<Double> valueGetter;
    private Supplier<Boolean> toSave;

    public Value(String valueName, 
        Supplier<Double> valueGetter, Supplier<Boolean> toSave) {
        this.valueName = valueName;
        this.valueGetter = valueGetter;
        this.toSave = toSave;
    }

    public Double getValue() {
        return valueGetter.get();
    }

    public boolean toSave() {
        return toSave.get();
    }

    public String getValueName() {
        return valueName;
    }
}
