package com.example.map

import File
import retrofit2.http.*

interface ApiService {
    // Diary API
    @GET("diary")
    suspend fun getAllDiaries(): List<Diary>

    @POST("diary/add/{keyword}")
    suspend fun createDiary(@Path("keyword") keyword: String): Diary

    @DELETE("diary/delete/{id}")
    suspend fun deleteDiary(@Path("id") id: Long): Boolean

    @PUT("diary/edit/{id}")
    suspend fun updateDiary(@Path("id") id: Long, @Body diary: Diary): Diary

    @GET("diary/user/{userId}/date/{date}")
    suspend fun getDiariesByUserAndDateRange(
        @Path("userId") userId: Long,
        @Path("date") date: String
    ): List<Diary>

    @GET("diary/user/{userId}/place/{placeId}")
    suspend fun getDiariesByUserAndPlace(
        @Path("userId") userId: Long,
        @Path("placeId") placeId: Long
    ): List<Diary>

    // File API
    @POST("file/upload")
    suspend fun uploadFile(@Body file: File): String

    // Place API
    @POST("place/add")
    suspend fun addPlace(@Body place: Place): Place

    @DELETE("place/delete/{id}")
    suspend fun deletePlace(@Path("id") id: Long): Boolean

    @PUT("place/edit")
    suspend fun editPlace(@Body place: Place): Place

    @GET("place/get/{id}")
    suspend fun getPlace(@Path("id") id: Long): Place

    @GET("place/getAll")
    suspend fun getAllPlaces(): List<Place>

    @GET("place/search")
    suspend fun searchPlaceByKeyword (@Query("keyword") keyword: String): List<Place>


    // Plan API
    @POST("plans/add")
    suspend fun createPlan(@Body plan: Plan): Plan

    @DELETE("plans/delete/{planId}")
    suspend fun deletePlan(@Path("planId") planId: Long): Boolean

    @PUT("plans/edit")
    suspend fun updatePlan(@Body plan: Plan): Plan

    @GET("plans/getAll")
    suspend fun getAllPlans(): List<Plan>

    @GET("plans/user/{userId}")
    suspend fun getUserPlans(@Path("userId") userId: Long): List<Plan>

    @GET("plans/view/{planId}")
    suspend fun getPlanById(@Path("planId") planId: Long): Plan

    // User API
    @POST("user/add")
    suspend fun registerUser(@Body user: User): User

    @DELETE("user/delete/{id}")
    suspend fun deleteUser(@Path("id") id: Long): Boolean

    @PUT("user/edit")
    suspend fun updateUser(@Body user: User): User

    @GET("user/get/{id}")
    suspend fun getUserById(@Path("id") id: Long): User

    @GET("user/getAll")
    suspend fun getAllUsers(): List<User>



    // Rest Test Controller API
    @POST("add")
    suspend fun createDiary(): Diary

    @POST("makeDiary/{keyword}")
    suspend fun callApiHttp(@Path("keyword") keyword: String): Diary

    // Plan Places API
    @POST("planplaces/add")
    suspend fun createPlanPlace(@Body planPlace: PlanPlace): PlanPlace

    @GET("planplaces/calculateTime/{id}")
    suspend fun calculateTime(@Path("id") id: Long): Long

    @DELETE("planplaces/delete/{id}")
    suspend fun deletePlanPlace(@Path("id") id: Long): Boolean

    @GET("planplaces/getAll")
    suspend fun getAllPlanPlaces(): List<PlanPlace>

    @GET("planplaces/search/by-plan/{planId}")
    suspend fun getPlanPlacesByPlanId(@Path("planId") planId: Long): List<PlanPlace>


    @GET("planplaces/search/by-place/{placeId}")
    suspend fun getPlanPlacesByPlaceId(@Path("placeId") placeId: Long): List<PlanPlace>


}
