package com.nemo.neplan.service;

import com.nemo.neplan.model.Plan;
import com.nemo.neplan.model.PlanPlace;

import java.util.List;

public interface PlanPlaceService {
    List<PlanPlace> getAllPlanPlaces();
    List<PlanPlace> getPlanPlacesByPlanId(Long planId);
    List<PlanPlace> getPlanPlacesByPlaceId(Long placeId);
    PlanPlace createPlanPlace(PlanPlace planPlace);
    void deletePlanPlace(Long id);





    void editPlanPlace(PlanPlace planPlace);


}
