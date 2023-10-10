package com.nemo.neplan.controller;

import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.Plan;
import com.nemo.neplan.model.PlanPlace;
import com.nemo.neplan.repository.PlanPlaceRepository;
import com.nemo.neplan.service.PlanPlaceService;
import com.nemo.neplan.service.PlanService;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpHeaders;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.util.UriComponentsBuilder;

import com.nemo.neplan.model.KakaoApiResponse;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/planplaces")
@Api(tags = "플랜 속 경유지 관련 API", description = "경유지 관리 API")
public class PlanPlaceController {

    @Autowired
    private PlanPlaceService planPlaceService;

    @GetMapping("/getAll")
    public ResponseEntity<List<PlanPlace>> getAllPlanPlaces() {
        List<PlanPlace> planPlaces = planPlaceService.getAllPlanPlaces();
        return ResponseEntity.ok(planPlaces);
    }

    @GetMapping("/search/by-plan/{planId}")
    public ResponseEntity<List<PlanPlace>> getPlanPlacesByPlanId(@PathVariable Long planId) {
        List<PlanPlace> planPlaces = planPlaceService.getPlanPlacesByPlanId(planId);
        return ResponseEntity.ok(planPlaces);

    }

    @GetMapping("/search/by-place/{placeId}")
    public ResponseEntity<List<PlanPlace>> getPlanPlacesByPlaceId(@PathVariable Long placeId) {
        List<PlanPlace> planPlaces = planPlaceService.getPlanPlacesByPlaceId(placeId);
        return ResponseEntity.ok(planPlaces);
    }

    @PostMapping("/add")
    public ResponseEntity<PlanPlace> createPlanPlace(@RequestBody PlanPlace planPlace) {
        System.out.println("경유지 추가 요청이 들어왔어요");
        PlanPlace createdPlanPlace = planPlaceService.createPlanPlace(planPlace);
        System.out.println("경유지 추가 완료");
        return ResponseEntity.ok(createdPlanPlace);
    }

    @DeleteMapping("/delete/{id}")
    public ResponseEntity<Void> deletePlanPlace(@PathVariable Long id) {
        planPlaceService.deletePlanPlace(id);
        return ResponseEntity.noContent().build();
    }


    @GetMapping("/getPlacesByPlanId/{planId}")
    public ResponseEntity<Map<String, double[][]>> GettingPlaces(
            @PathVariable Long planId) {

        //,
        //            @RequestParam int who,
        //            @RequestParam int where

        List<PlanPlace> planPlaces = planPlaceService.getPlanPlacesByPlanId(planId);
        // Update placeOrder values

        double[][] coordination=new double[planPlaces.size()][2];

            for (int i = 0; i < planPlaces.size(); i++) {

                PlanPlace p=planPlaces.get(i);
                coordination[i][0]=Double.parseDouble(p.getPlace().getX());
                coordination[i][1]=Double.parseDouble(p.getPlace().getY());

            }

        Map<String, double[][]> response = new HashMap<>();
        response.put("coordination", coordination);

        return ResponseEntity.ok(response);


//        if (who >= 0 && who < planPlaces.size() && where >= 0 && where < planPlaces.size()) {
//            PlanPlace planPlaceToMove = planPlaces.get(who);
//            planPlaces.remove(who); // Remove from original position
//            planPlaces.add(where, planPlaceToMove); // Add to new position

//            // Update placeOrder values
//            for (int i = 0; i < planPlaces.size(); i++) {
////                planPlaces.get(i).setPlaceOrder(i);
//                planPlaceService.createPlanPlace(planPlaces.get(i));
//            }
//
//            return ResponseEntity.ok(planPlaces);
//        } else {
//            return ResponseEntity.badRequest().build();
//        }
    }




    class TimeDuration{
        int distance; //미터
        int duration; //초

        public int getDistance() {
            return distance;
        }

        public void setDistance(int distance) {
            this.distance = distance;
        }

        public int getDuration() {
            return duration;
        }

        public void setDuration(int duration) {
            this.duration = duration;
        }
    }

    @Value("${kakao.api.key}")
    private String apiKey;

    @Autowired
    private PlanService planService;



    @GetMapping("/calculateTime/{id}")
    public ResponseEntity<TimeDuration> calculateTime(@PathVariable Long id) {
        Plan plan = planService.getPlanById(id);

        LocalDateTime departureDateTime = LocalDateTime.now().plusDays(1);

        // Convert LocalDateTime to "yyyyMMddHHmm" format
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy.MM.dd'T'HH:mm");
        String formattedDepartureTime = departureDateTime.format(formatter);

        List<PlanPlace> planPlaces = planPlaceService.getPlanPlacesByPlanId(id);
        System.out.println("찾은 장소의 개수: " + planPlaces.size());
        String waypointsString = buildWaypointsString(planPlaces);
        RestTemplate restTemplate = new RestTemplate();

        HttpHeaders headers = new HttpHeaders();
        headers.set("Authorization", "KakaoAK 45212e7b98de5fc6be36009fadc07ab5");
        headers.set("Content-Type", "application/json");

        String apiUrl = "https://apis-navi.kakaomobility.com/v1/future/directions";
        UriComponentsBuilder builder = UriComponentsBuilder.fromHttpUrl(apiUrl)
                .queryParam("departure_time", formattedDepartureTime);

        if (planPlaces.size() > 7) {
            // Set origin and destination
            builder.queryParam("origin", planPlaces.get(6).getPlace().getX() + "," +
                            planPlaces.get(6).getPlace().getY())
                    .queryParam("destination", planPlaces.get(planPlaces.size() - 1).getPlace().getX() + "," +
                            planPlaces.get(planPlaces.size() - 1).getPlace().getY());

            // Set waypoints
            String intermediateWaypoints = buildWaypointsString(planPlaces.subList(0, planPlaces.size() - 1));
            builder.queryParam("waypoints", intermediateWaypoints);
        } else {
            // Set origin and destination for size <= 7
            builder.queryParam("origin", planPlaces.get(0).getPlace().getX() + "," +
                            planPlaces.get(0).getPlace().getY())
                    .queryParam("destination", planPlaces.get(planPlaces.size() - 1).getPlace().getX() + "," +
                            planPlaces.get(planPlaces.size() - 1).getPlace().getY());

            // Set waypoints as empty for size <= 7
            builder.queryParam("waypoints", "");
        }

        KakaoApiResponse response = restTemplate.getForObject(builder.toUriString(), KakaoApiResponse.class);

        TimeDuration timeDuration = new TimeDuration();
        timeDuration.setDistance(response.getDistance());
        timeDuration.setDuration(response.getDuration());

        return ResponseEntity.ok(timeDuration);
    }

    private String buildWaypointsString(List<PlanPlace> planPlaces) {
        StringBuilder waypointsBuilder = new StringBuilder();
        for (int i = 1; i < planPlaces.size() - 1; i++) {
            PlanPlace place = planPlaces.get(i);
            String waypoint = place.getPlace().getX() + "," + place.getPlace().getY();
            waypointsBuilder.append(waypoint);
            if (i < planPlaces.size() - 2) {
                waypointsBuilder.append(" | ");
            }
        }
        return waypointsBuilder.toString();
    }
}
