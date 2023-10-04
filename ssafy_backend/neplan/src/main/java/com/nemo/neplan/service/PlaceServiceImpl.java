package com.nemo.neplan.service;

import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.PlaceType;
import com.nemo.neplan.model.User;
import com.nemo.neplan.repository.PlaceRepository;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.*;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestTemplate;
import com.nemo.neplan.model.Place;
import com.nemo.neplan.repository.PlaceRepository;import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

@Service
@Api(tags = "장소 관리", description = "장소 관리를 위한 API들")
public class PlaceServiceImpl implements PlaceService {

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

    @Override
    public List<Place> searchPlaceByKeyword(String keyword) {
        List<Place> places = new ArrayList<>();
        try {

            String apiUrl = "https://dapi.kakao.com/v2/local/search/keyword.json?query=" + keyword;
            String apiKey = "898c3cb2e137819932a3fed6179caee0";
            HttpHeaders headers = new HttpHeaders();
            headers.set("Authorization", "KakaoAK " + apiKey);


            ResponseEntity<String> response = new RestTemplate().exchange(apiUrl, HttpMethod.GET, new HttpEntity<>(headers), String.class);
            String responseBody = response.getBody();

            System.out.println("장소 검색 결과 : "+responseBody);

            // Parse responseBody and extract place information
            ObjectMapper objectMapper = new ObjectMapper();
            JsonNode rootNode = objectMapper.readTree(responseBody);
            JsonNode documentsNode = rootNode.get("documents");

            if (documentsNode != null && documentsNode.isArray()) {
                for (JsonNode placeNode : documentsNode) {
                    Place place = extractPlaceInfo(placeNode);
                    if (place != null) {
                        places.add(place);
                    }
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return places;

    }

    private Place extractPlaceInfo(JsonNode placeNode) {
        Place place = null;

        try {
            String name = placeNode.path("place_name").asText();
            String address = placeNode.path("address_name").asText();
            String latitude = placeNode.path("y").asText();
            String longitude = placeNode.path("x").asText();

            place = new Place(name, address, latitude, longitude);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return place;
    }



    @Override
    public Place searchAndSavePlace(String keyword) {
        try {

            String apiUrl = "https://dapi.kakao.com/v2/local/search/keyword.json?query=" + keyword;
            String apiKey = "898c3cb2e137819932a3fed6179caee0";
            HttpHeaders headers = new HttpHeaders();
            headers.set("Authorization", "KakaoAK " + apiKey);


            ResponseEntity<String> response = new RestTemplate().exchange(apiUrl, HttpMethod.GET, new HttpEntity<>(headers), String.class);
            String responseBody = response.getBody();

            System.out.println("장소 검색 결과 : "+responseBody);

            // Parse responseBody and extract the first place information
            ObjectMapper objectMapper = new ObjectMapper();
            JsonNode rootNode = objectMapper.readTree(responseBody);
            JsonNode documentsNode = rootNode.get("documents");
            if (documentsNode != null && documentsNode.size() > 0) {
                JsonNode firstPlaceNode = documentsNode.get(0);
                String name = firstPlaceNode.get("place_name").asText();
                String phone = firstPlaceNode.get("phone").asText();
                String address = firstPlaceNode.get("address_name").asText();
                String x = firstPlaceNode.get("x").asText();
                String y = firstPlaceNode.get("y").asText();
                String placeTypeString = firstPlaceNode.get("category_group_code").asText();
                PlaceType placeType = PlaceType.fromString(placeTypeString);

                System.out.println("카ㅏ테고리 코드 : "+placeTypeString);
                System.out.println("카체고리 타입 : "+placeType);

                // Check if the place with the same name already exists in the database
                Place existingPlace = placeRepository.findByName(name);
                if (existingPlace != null) {
                    return existingPlace; // If the place already exists, return it
                }

                // If the place doesn't exist in the DB, create a new Place object and save it to the DB
                Place newPlace = new Place();
                newPlace.setName(name);
                newPlace.setPhone(phone);
                newPlace.setAddress(address);
                newPlace.setPlaceType(placeType);
                newPlace.setX(x);
                newPlace.setY(y);

                return placeRepository.save(newPlace); // Save the new place to the database and return it
            }
        } catch (JsonProcessingException e) {
            e.printStackTrace(); // Handle JsonProcessingException
        } catch (Exception e) {
            e.printStackTrace(); // Handle other exceptions (including JsonMappingException)
        }

        return null; // If an exception occurs or no place information was found, return null
    }






}








