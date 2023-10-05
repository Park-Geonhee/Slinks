package com.example.map

import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
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
     fun GettingPlaces(@Path("planId") planId:Long):Call<List<Coordination>>

     @GET("place/search")
    fun searchPlaceByKeyword(@Body keyword:String):Call<List<Place>>


    @GET("diary/user/{userId}/place/{placeId}")
    fun getDiariesByUserAndPlace(@Path("userId") userId:Long,
                                 @Path("placeId") placeId: Long):Call<List<Diary>>
}
