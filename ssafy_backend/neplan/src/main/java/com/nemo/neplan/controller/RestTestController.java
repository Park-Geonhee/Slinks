package com.nemo.neplan.controller;


import com.nemo.neplan.model.Diary;
import com.nemo.neplan.service.DiaryService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.*;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.client.RestTemplate;
import java.text.SimpleDateFormat;
import java.util.Date;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

@RestController
public class RestTestController {


    @Autowired
    DiaryService diaryService;


    @PostMapping("/add")
    public ResponseEntity<Diary> createDiary(@RequestBody Diary diary) {
        Diary createdDiary = diaryService.createDiary(diary);
        return ResponseEntity.status(HttpStatus.CREATED).body(createdDiary);
    }

    @PostMapping("/makeDiary/{keyword}")
    public String callapihttp(@PathVariable String keyword) {

        String OPENAI_API_KEY = "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";

        // 현재 시간을 얻어오기
        SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
        Date date = new Date();
        String currentTime = formatter.format(date);

        try {
            // OpenAI API에 보낼 요청 데이터 준비
            String requestData = "{\"model\": \"gpt-3.5-turbo\", \"messages\": [{\"role\": \"user\", \"content\": \"주어진 시각과 검색 키워드를 활용해 100자 이내의 일기를 작성해줘. 시각: " + currentTime + ", 키워드: " + keyword + "\"}], \"temperature\": 0.7}";

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


            // API 응답을 그대로 반환
            return responseEntity.getBody();
        } catch (Exception e) {
            e.printStackTrace();
            return "API 호출 중 오류가 발생했습니다.";
        }
    }
}



