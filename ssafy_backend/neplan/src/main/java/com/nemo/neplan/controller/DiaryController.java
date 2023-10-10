package com.nemo.neplan.controller;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.nemo.neplan.model.Diary;
import com.nemo.neplan.model.File;
import com.nemo.neplan.model.Place;
import com.nemo.neplan.model.User;
import com.nemo.neplan.service.DiaryService;
import com.nemo.neplan.service.FileService;
import com.nemo.neplan.service.PlaceService;
import io.swagger.annotations.Api;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.http.*;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;
import org.springframework.web.multipart.MultipartFile;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
@RestController
@Api(tags = "Diary 관련 API", description = "일기 관리 API")
@RequestMapping("/diary")
public class DiaryController {

    @Autowired
    private final DiaryService diaryService;

    @Autowired
    private final FileService fileService;

    @Autowired
    private PlaceService placeService;

    @Autowired
    public DiaryController(DiaryService diaryService, FileService fileService) {
        this.diaryService = diaryService;
        this.fileService = fileService;
    }

    @GetMapping("/getAll")
    public ResponseEntity<List<Diary>> getAllDiaries() {
        List<Diary> diaries = diaryService.getAllDiaries();
        return ResponseEntity.ok(diaries);
    }

    @GetMapping("/user/{userId}/date/{date}")
    public ResponseEntity<List<Diary>> getDiariesByUserAndDateRange(
            @PathVariable Long userId,
            @PathVariable @DateTimeFormat(pattern = "yyyyMMdd") String date) {

        System.out.println("오늘 다이어리 조회합니다잉");
        List<Diary> diaries = diaryService.getDiariesByUserAndCreatedDate(userId, date);
        return ResponseEntity.ok(diaries);
    }

    @GetMapping("/user/{userId}/place/{placeId}")
    public ResponseEntity<List<Diary>> getDiariesByUserAndPlace(
            @PathVariable Long userId,
            @PathVariable Long placeId) {
        // userId와 placeId를 이용하여 User와 Place 객체 가져오기
        List<Diary> diaries = diaryService.getDiariesByUserAndPlace(userId, placeId);
        return ResponseEntity.ok(diaries);
    }


    @GetMapping("/user/{userId}")
    public ResponseEntity<List<Diary>> getDiaryPlacesByUserId(@PathVariable Long userId){
        List<Diary> diaries=diaryService.getDiaryByUserId(userId);

//        List<Place> places=new ArrayList<>();
//        for(Diary d: diaries){
//            places.add(d.getPlace());
//        }
        return ResponseEntity.ok(diaries);
    }

    @PostMapping("/add")
    public ResponseEntity<Diary> createDiary(
            @RequestParam("file") MultipartFile file,
            @RequestParam("keyword") String keyword,
            @RequestParam("userId") String userId
         ) {


        try {
            Long userIdl = Long.parseLong(userId);
            // 이미지 파일 업로드
            long fileId = storeFile(file);

            // 장소 검색 및 저장
//            String keyword="서울특별시 강남구 테헤란로 212";
            Place place = placeService.searchAndSavePlace(keyword);

            System.out.println("일기에 저장할 장소 정보 : "+place.toString());

            User user=new User();
            user.setId(userIdl);

            // 일기 데이터 생성 및 저장
            Diary diary = new Diary();
            diary.setUser(user);
            diary.setPlace(place);

            File ff=new File();
            ff.setId(fileId);

            diary.setFile(ff);
//

            String OPENAI_API_KEY = "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";

            // 현재 시간을 얻어오기
            SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
            Date date = new Date();
            String currentTime = formatter.format(date);

            try {
                // OpenAI API에 보낼 요청 데이터 준비
                String requestData = "{\"model\": \"gpt-3.5-turbo\", \"messages\": [{\"role\": \"user\", \"content\": \"주어진 시각과 장소 검색 키워드를 가지고 100자 이내의 일기를 작성해줘. 이때, 날짜와 시각은 직접적으로 언급하지 말아줘. :) 같은 귀여운 이모티콘도 넣어줘. 시각: " + currentTime + ", 키워드: " + keyword + "\"}], \"temperature\": 0.7}";

                // API 요청을 위한 헤더 설정
                HttpHeaders headers = new HttpHeaders();
                headers.setContentType(MediaType.APPLICATION_JSON);
                headers.set("Authorization", "Bearer " + OPENAI_API_KEY);

                // API 호출
                HttpEntity<String> requestEntity = new HttpEntity<>(requestData, headers);
                ResponseEntity<String> responseEntity = new RestTemplate().postForEntity("https://api.openai.com/v1/chat/completions", requestEntity, String.class);


                // OpenAI API 응답에서 content 필드 추출
                ObjectMapper objectMapper = new ObjectMapper();
                JsonNode rootNode = objectMapper.readTree(responseEntity.getBody());
                String content = rootNode.get("choices").get(0).get("message").get("content").asText();

                diary.setContent(content);

                System.out.println("일기 내용 : "+content);

            } catch (Exception e) {
                e.printStackTrace();

            }

            System.out.println("일기 등록 시작");
            Diary createdDiary = diaryService.createDiary(diary);
            System.out.println("일기 등록 완료");


            return ResponseEntity.status(HttpStatus.CREATED).body(createdDiary);
        } catch (Exception e) {
            e.printStackTrace();
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).build();
        }
    }

    private Long storeFile(MultipartFile file) {
        try {
            // 파일 업로드 로직 구현 (이미지 파일을 서버에 저장하고 파일 Id를 반환)
            Long fileId = fileService.uploadFiles(file);
            return fileId;
        } catch (IOException e) {
            e.printStackTrace(); // 예외 처리 로직 추가 (예: 로깅)
            return null; // 또는 적절한 오류 메시지를 반환하여 클라이언트에게 알림
        } catch (Exception e) {
            e.printStackTrace(); // 예외 처리 로직 추가 (예: 로깅)
            return null; // 또는 적절한 오류 메시지를 반환하여 클라이언트에게 알림
        }
    }


    @PutMapping("/edit/{id}")
    public ResponseEntity<Diary> updateDiary(
            @PathVariable Long id,
            @RequestBody Diary diary) {
        Diary updatedDiary = diaryService.updateDiary(diary);
        return ResponseEntity.ok(updatedDiary);
    }

    @DeleteMapping("/delete/{id}")
    public ResponseEntity<Void> deleteDiary(@PathVariable Long id) {
        diaryService.deleteDiary(id);
        return ResponseEntity.noContent().build();
    }
}
