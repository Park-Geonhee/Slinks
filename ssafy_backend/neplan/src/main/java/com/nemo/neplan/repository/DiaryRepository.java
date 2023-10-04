package com.nemo.neplan.repository;

import com.nemo.neplan.model.Diary;
import com.nemo.neplan.model.File;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface DiaryRepository  extends JpaRepository<Diary, Long> {

    @Query("SELECT d FROM Diary d WHERE d.user.id = :userId AND d.createdDate LIKE CONCAT(:datePrefix, '%')")
    List<Diary> findByUserIdAndCreatedDate(@Param("userId") Long userId, @Param("datePrefix") String datePrefix);


    List<Diary> findByPlaceId(Long placeId);
    List<Diary> findByUserId(Long userId);
    List<Diary> findByUserIdAndPlaceId(Long userId, Long placeId);
}
