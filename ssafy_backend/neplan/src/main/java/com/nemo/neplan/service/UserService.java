package com.nemo.neplan.service;

import com.nemo.neplan.model.User;

import java.util.List;

public interface UserService {
    public User saveUser(User user);
    public List<User> getAllUser();

    public User getUser(long id);
    public User modifyUser(User user);
    public int withdrawUser(long id);

    public User login(String email,String password);
}
