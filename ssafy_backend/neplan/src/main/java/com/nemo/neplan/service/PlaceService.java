package com.nemo.neplan.service;

import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.User;

import java.util.List;

public interface PlaceService {

    public Place savePlace(Place place);
    public List<Place> getAllPlace();

    public Place getPlace(long id);
    public Place modifyPlace(Place place);
    public int deletePlace(long id);

    List<Place> searchByAddress(String address);

}
