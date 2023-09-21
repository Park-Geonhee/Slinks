package com.nemo.neplan.model;

public enum PlaceType {
    CAFE("Cafe"),
    RESTAURANT("Restaurant"),
    LANDMARK("Landmark"),
    HOSTEL("Hostel"),
    CULTURE("Culture"),
    MART("Mart"),
    ETC("Etc");

    private final String value;

    PlaceType(String value) {
        this.value = value;
    }

    public String getValue() {
        return value;
    }
}
