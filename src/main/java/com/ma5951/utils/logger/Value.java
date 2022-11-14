package com.ma5951.utils.logger;

import java.util.function.Supplier;

public class Value<T> {
    private String valueName;
    private Supplier<T> valueGetter;

    public Value(String valueName, Supplier<T> valueGetter) {
        this.valueName = valueName;
        this.valueGetter = valueGetter;
    }

    public T getValue() {
        return valueGetter.get();
    }

    public String getValueName() {
        return valueName;
    }
}
