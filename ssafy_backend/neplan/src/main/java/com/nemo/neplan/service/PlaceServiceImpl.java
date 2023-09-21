package com.nemo.neplan.service;

import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.User;
import com.nemo.neplan.repository.PlaceRepository;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;

@Service
@Api(tags = "장소 관리", description = "장소 관리를 위한 API들")
public class PlaceServiceImpl implements PlaceService{

    @Autowired
    private PlaceRepository placeRepository;


    @Override
    public Place savePlace(Place place) {


        return placeRepository.save(place);
    }

    @Override
    public List<Place> getAllPlace() {
        return placeRepository.findAll();
    }

    @Override
    public Place getPlace(long id) {
        Optional<Place> placeOptional = placeRepository.findById(id);
        return placeOptional.orElse(null);
    }


    @Override
    public Place modifyPlace(Place place) {
        return placeRepository.save(place);
    }

    @Override
    public int deletePlace(long id) {
        try {
            placeRepository.deleteById(id);
            return 1;
        } catch (Exception e) {
            return 0;
        }
    }

    @Override
    public List<Place> searchByAddress(String address) {
        return placeRepository.searchByAddressContaining(address);
    }


}
