package com.nemo.neplan.service;


import com.nemo.neplan.exception.UserNotFoundException;
import com.nemo.neplan.model.User;
import com.nemo.neplan.repository.UserRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import io.swagger.annotations.Api;
import io.swagger.annotations.ApiOperation;
import java.util.List;
import java.util.Optional;

@Service
@Api(tags = "사용자 관리", description = "사용자 관리를 위한 API들")
public class UserServiceImpl implements UserService{

    @Autowired
    private UserRepository userRepository;

    @Override
    @ApiOperation("새로운 사용자 저장")
    public User saveUser(User user) {
        return userRepository.save(user);
    }

    @Override
    @ApiOperation("모든 사용자 목록 조회")
    public List<User> getAllUser() {
        return userRepository.findAll();
    }

    @Override
    @ApiOperation("사용자 ID로 사용자 조회")
    public User getUser(long id) {
        Optional<User> optionalUser = userRepository.findById(id);

        return optionalUser.orElseThrow(() -> new UserNotFoundException("User with ID " + id + " not found"));
    }

    @Override
    @ApiOperation("기존 사용자 수정")
    public User modifyUser(User user) {
        Long userId = user.getId(); // 사용자의 ID를 가져옵니다.

        Optional<User> optionalUser = userRepository.findById(userId);

        if (optionalUser.isPresent()) {
            User existingUser = optionalUser.get();
            // 사용자가 존재하는 경우 처리 로직

            // user 객체의 필드를 기반으로 existingUser의 정보를 업데이트합니다.
            existingUser.setEmail(user.getEmail());
            existingUser.setAddress(user.getAddress());

            // 업데이트된 사용자를 저장하고 반환합니다.
            User updatedUser = userRepository.save(existingUser);
            return updatedUser;
        } else {
            //사용자가 존재하지 않는다면 예외처리를 합니다.
            throw new UserNotFoundException("User with ID " + userId + " not found");
        }
    }

    @Override
    @ApiOperation("사용자 ID로 사용자 삭제")
    public int withdrawUser(long id) {
        Optional<User> optionalUser = userRepository.findById(id);

        if (optionalUser.isPresent()) {
            userRepository.delete(optionalUser.get());
            return 1; // 성공적으로 삭제되었음을 나타내는 값
        } else {
            throw new UserNotFoundException("ID가 " + id + "인 사용자를 찾을 수 없어 삭제할 수 없습니다");
        }
    }

    @Override
    public User login(String email, String password) {
        User user = userRepository.findByEmail(email);
        if (user != null && user.getPassword().equals(password)) {
            return user;
        }
        return null;
    }
}