plugins { id("com.android.application") version "9.5.3" }

android {
    namespace = "io.stillflow.runner"
    compileSdk { version = release(37) { minorApiLevel = 2 } }
    buildToolsVersion = "37.0.0"
    defaultConfig {
        applicationId = "io.stillflow.runner"
        minSdk = 29
        targetSdk = 37
        versionCode = 1
        versionName = "1"
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    buildTypes {
        release {
            isMinifyEnabled = true
            isShrinkResources = true
            // A local tool: run-as transfers files without shared-storage permissions.
            isDebuggable = true
            signingConfig = signingConfigs.getByName("debug")
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"))
        }
    }
    lint {
        warningsAsErrors = true
        abortOnError = true
        disable += "ChromeOsAbiSupport"
    }
}
dependencies {
    implementation(files(providers.gradleProperty("stillflow.aar")
        .getOrElse("../../api/android/build/outputs/aar/stillflow-release.aar")))
}
