package com.nemo.neplan.controller;


import com.nemo.neplan.model.Place;
import com.nemo.neplan.service.PlaceService;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@Api(tags = "Place 관련 API", description = "장소 관리 API")
@RequestMapping("/place")
public class PlaceController {

    @Autowired
    private PlaceService placeService;

    @PostMapping("/add")
    public ResponseEntity<String> addPlace(@RequestBody Place place) {
        placeService.savePlace(place);
        return new ResponseEntity<>("장소 등록이 완료되었습니다.", HttpStatus.OK);
    }

    @GetMapping("/getAll")
    public ResponseEntity<List<Place>> getAllPlaces() {
        List<Place> places = placeService.getAllPlace();
        return new ResponseEntity<>(places, HttpStatus.OK);
    }

    @GetMapping("/get/{id}")
    public ResponseEntity<Place> getPlace(@PathVariable("id") long id) {
        Place place = placeService.getPlace(id);
        if (place != null) {
            return new ResponseEntity<>(place, HttpStatus.OK);
        } else {
            return new ResponseEntity<>(HttpStatus.NOT_FOUND);
        }
    }

    @PutMapping("/edit")
    public ResponseEntity<Place> editPlace(@RequestBody Place place) {
        Place updatedPlace = placeService.modifyPlace(place);
        if (updatedPlace != null) {
            return new ResponseEntity<>(updatedPlace, HttpStatus.OK);
        } else {
            return new ResponseEntity<>(HttpStatus.NOT_FOUND);
        }
    }

    @DeleteMapping("/delete/{id}")
    public ResponseEntity<String> deletePlace(@PathVariable("id") long id) {
        int result = placeService.deletePlace(id);
        if (result == 1) {
            return new ResponseEntity<>("장소가 삭제되었습니다.", HttpStatus.OK);
        } else {
            return new ResponseEntity<>("장소를 찾을 수 없습니다.", HttpStatus.NOT_FOUND);
        }
    }

    @GetMapping("/search")
    public ResponseEntity<List<Place>> searchPlaces(
            @RequestParam(required = false) String city,
            @RequestParam(required = false) String district,
            @RequestParam(required = false) String neighborhood,
            @RequestParam(required = false) String street,
            @RequestParam(required = false) String buildingNumber
    ) {
        List<Place> places;

        if (buildingNumber != null) {
            // 모든 검색어가 제공된 경우
            places = placeService.searchByCityAndDistrictAndNeighborhoodAndStreetAndBuildingNumber(
                    city, district, neighborhood, street, buildingNumber);
        } else if (street != null) {
            // 건물번호를 제외한 검색어가 제공된 경우
            places = placeService.searchByCityAndDistrictAndNeighborhoodAndStreet(
                    city, district, neighborhood, street);
        } else if (neighborhood != null) {
            // 거리와 건물번호를 제외한 검색어가 제공된 경우
            places = placeService.searchByCityAndDistrictAndNeighborhood(
                    city, district, neighborhood);
        } else if (district != null) {
            // 동네부터 검색어가 제공된 경우
            places = placeService.searchByCityAndDistrict(city, district);
        } else if (city != null) {
            // 도시만 검색어가 제공된 경우
            places = placeService.searchByCity(city);
        } else {
            // 검색어 없이 요청이 들어온 경우
            return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
        }

        return new ResponseEntity<>(places, HttpStatus.OK);
    }




}
