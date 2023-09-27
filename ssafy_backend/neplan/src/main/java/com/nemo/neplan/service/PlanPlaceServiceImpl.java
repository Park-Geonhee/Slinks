package com.nemo.neplan.service;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.model.PlanPlace;
import com.nemo.neplan.repository.PlanPlaceRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.Optional;

@Service
public class PlanPlaceServiceImpl implements PlanPlaceService {

    @Autowired
    private PlanPlaceRepository planPlaceRepository;

    @Override
    public List<PlanPlace> getAllPlanPlaces() {
        return planPlaceRepository.findAll();
    }

    @Override
    public List<PlanPlace> getPlanPlacesByPlanId(Long planId) {
        return planPlaceRepository.findByPlanId(planId);
    }

    @Override
    public List<PlanPlace> getPlanPlacesByPlaceId(Long placeId) {
        return planPlaceRepository.findByPlaceId(placeId);
    }

    @Override
    public PlanPlace createPlanPlace(PlanPlace planPlace) {
        int count = planPlaceRepository.countByPlanId(planPlace.getPlan().getId());
        System.out.println("현재 플랜에는 "+count+"개의 장소가 저장되어있습니다");
//        planPlace.setPlaceOrder(count);

        return planPlaceRepository.save(planPlace);
    }


    @Override
    public void deletePlanPlace(Long id) {
        planPlaceRepository.deleteById(id);
    }

    @Override
    public void editPlanPlace(PlanPlace planPlace) {
        Optional<PlanPlace> optionalPlanPlace = planPlaceRepository.findById(planPlace.getId());

        if (optionalPlanPlace.isPresent()) {
            PlanPlace existingPlanPlace = optionalPlanPlace.get();
//            existingPlanPlace.setPlaceOrder(planPlace.getPlaceOrder());

            planPlaceRepository.save(existingPlanPlace);
        } else {
            // Handle the case when the PlanPlace is not found
            // You can throw an exception or handle it based on your application logic...
        }
    }


}
