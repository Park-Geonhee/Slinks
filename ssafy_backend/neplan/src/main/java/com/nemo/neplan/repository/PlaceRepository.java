package com.nemo.neplan.repository;


import com.nemo.neplan.model.Place;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;

public interface PlaceRepository extends JpaRepository<Place, Long> {
    // 주소를 LIKE 조건으로 검색하는 쿼리
    @Query("SELECT p FROM Place p WHERE p.address LIKE %:address%")
    List<Place> searchByAddressContaining(String address);


}
