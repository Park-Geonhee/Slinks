pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()

        maven(url = "https://jcenter.bintray.com/") // JCenter 저장소 추가
        // ROS Maven 저장소 추가
        maven(url = "https://github.com/rosjava/rosjava_mvn_repo/raw/master")
    }
}

rootProject.name = "googlemap"
include(":app")
 