package com.nemo.neplan.controller;

import com.nemo.neplan.model.User;
import com.nemo.neplan.service.UserService;
import io.swagger.annotations.Api;
import io.swagger.annotations.ApiOperation;
import io.swagger.annotations.ApiResponse;
import io.swagger.annotations.ApiResponses;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import javax.websocket.server.PathParam;
import java.util.List;

@RestController
@Api(tags = "User 관련 API", description = "사용자 관리 API")
@RequestMapping("/user")
public class UserController {
    @Autowired
    private UserService userService;

    @GetMapping("/getAll")
    @ApiOperation(value = "모든 사용자 조회", notes = "모든 사용자 정보를 조회합니다.")
    public ResponseEntity<List<User>> getAllUsers() {
        List<User> users = userService.getAllUser();
        return new ResponseEntity<>(users, HttpStatus.OK);
    }

    @GetMapping("/get/{id}")
    @ApiOperation(value = "특정 사용자 조회", notes = "주어진 ID에 해당하는 사용자 정보를 조회합니다.")
    @ApiResponses({
        @ApiResponse(code = 200, message = "성공적으로 사용자 정보 조회"),
        @ApiResponse(code = 404, message = "주어진 ID에 해당하는 사용자 없음")
    })
    public ResponseEntity<User> getUser(@PathVariable("id") Long id) {
        User user = userService.getUser(id);
        if (user != null) {
            return new ResponseEntity<>(user, HttpStatus.OK);
        } else {
            return new ResponseEntity<>(HttpStatus.NOT_FOUND);
        }
    }

    @PostMapping("/add")
    @ApiOperation(value = "사용자 등록", notes = "새로운 사용자를 등록합니다.")
    @ApiResponse(code = 201, message = "성공적으로 사용자 등록 완료")
    public ResponseEntity<String> addUser(@RequestBody User user) {
        userService.saveUser(user);
        return new ResponseEntity<>("사용자 등록이 완료되었습니다", HttpStatus.CREATED);
    }

    @PutMapping("/edit")
    @ApiOperation(value = "사용자 정보 수정", notes = "사용자 정보를 수정합니다.")
    @ApiResponses({
            @ApiResponse(code = 200, message = "성공적으로 사용자 정보 수정 완료"),
            @ApiResponse(code = 404, message = "주어진 ID에 해당하는 사용자 없음")
    })
    public ResponseEntity<User> editUser(@RequestBody User user) {
        User existingUser = userService.getUser(user.getId());
        if (existingUser == null) {
            return new ResponseEntity<>(HttpStatus.NOT_FOUND);
        } else {
            // Update user's information here
            userService.modifyUser(user);
            return new ResponseEntity<>(user, HttpStatus.OK);
        }
    }

    @DeleteMapping("/delete/{id}")
    @ApiOperation(value = "사용자 정보 삭제", notes = "사용자 정보를 삭제합니다.")
    @ApiResponses({
            @ApiResponse(code = 204, message = "성공적으로 사용자 정보 삭제 완료"),
            @ApiResponse(code = 404, message = "주어진 ID에 해당하는 사용자 없음")
    })
    public ResponseEntity<Void> deleteUser(@PathVariable("id") Long id) {
        User existingUser = userService.getUser(id);
        if (existingUser == null) {
            return new ResponseEntity<>(HttpStatus.NOT_FOUND);
        } else {
            userService.withdrawUser(id);
            return new ResponseEntity<>(HttpStatus.NO_CONTENT);
        }
    }

    @PostMapping("/login")
    public ResponseEntity<User> login(@RequestParam String email, @RequestParam String password) {
        User user = userService.login(email, password);
        if (user != null) {
//            // 로그인 성공
//            String token = generateToken(user); // 토큰 생성 예시
            return new ResponseEntity<>(user, HttpStatus.OK);
        } else {
            // 로그인 실패
            return new ResponseEntity<>( HttpStatus.UNAUTHORIZED);
        }
    }

}