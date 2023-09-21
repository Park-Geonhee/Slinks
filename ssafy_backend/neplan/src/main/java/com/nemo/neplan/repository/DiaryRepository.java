package com.nemo.neplan.repository;

import com.nemo.neplan.model.Diary;
import com.nemo.neplan.model.File;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface DiaryRepository  extends JpaRepository<Diary, Long> {
    List<Diary> findByUserIdAndCreatedDate(Long userId, String createdDate);
    List<Diary> findByPlaceId(Long placeId);
    List<Diary> findByUserId(Long userId);
    List<Diary> findByUserIdAndPlaceId(Long userId, Long placeId);
}
