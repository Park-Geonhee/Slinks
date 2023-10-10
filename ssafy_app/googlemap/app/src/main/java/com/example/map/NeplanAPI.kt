package com.example.map

import okhttp3.MultipartBody
import okhttp3.RequestBody
import retrofit2.Call
import retrofit2.Response
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.Multipart
import retrofit2.http.POST
import retrofit2.http.Part
import retrofit2.http.Path

interface NeplanAPI {
    @GET("diary/user/1/date/{date}")
 fun getDiaryByUserDay(@Path("date") date: String): Call<List<Diary>>

    @GET("diary/getAll")
     fun getAll():Call<List<Diary>>

    @GET("diary/user/{userId}")
     fun getDiaryByuserId(@Path("userId") userId: Long):Call<List<Diary>>


    @POST("planplaces/add")
    suspend fun createPlanPlace(@Body planPlace: PlanPlace): PlanPlace

    @GET("planplaces/getPlacesByPlanId/{planId}")
    fun GettingPlaces(@Path("planId") planId: Long): Call<Map<String, List<List<Double>>>>

     @GET("place/search")
    fun searchPlaceByKeyword(@Body keyword:String):Call<List<Place>>


    @GET("diary/user/{userId}/place/{placeId}")
    fun getDiariesByUserAndPlace(@Path("userId") userId:Long,
                                 @Path("placeId") placeId: Long):Call<List<Diary>>


    @GET("planplaces/search/by-plan/{planId}")
    suspend fun getPlanPlacesByPlanId(@Path("planId") planId: Long): Call<List<PlanPlace>>

    @POST("diary/add/{keyword}")
    suspend fun makeDiary(@Path("keyword") keyword: String): Diary

    @Multipart
    @POST("diary/add")
    suspend fun makeDiary(
        @Part("keyword") keyword: RequestBody,
        @Part file: MultipartBody.Part,
        @Part("userId") userId: RequestBody
    ): Response<Diary>


}
