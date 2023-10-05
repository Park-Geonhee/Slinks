package com.example.map

import File

data class Diary(
    val id: Long? = null,
    val content: String? = null,
    val user: User? = null,
    val place: Place? = null,
    val file: File? = null,
    val createdDate: String?=null,
    val modifiedDate: String?=null
)
