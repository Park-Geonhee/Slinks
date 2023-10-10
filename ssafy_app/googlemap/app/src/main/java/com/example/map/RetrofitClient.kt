package com.example.map

import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory

object RetrofitClient {

//    private const val BASE_URL = "http://175.209.87.132:8080/"
    private const val BASE_URL = "http://j9a701.p.ssafy.io/"

    private val getRetrofit by lazy{
        Retrofit.Builder()
            .baseUrl(BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
    }


    val getRetrofitService:NeplanAPI by lazy{
        getRetrofit.create(NeplanAPI::class.java)
    }

}